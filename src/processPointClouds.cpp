// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	for (; --maxIterations;)
	{
		// Randomly sample subset and fit line
		int idx1 = rand() % cloud->size();
		int idx2 = rand() % (cloud->size() - 1);
		int idx3 = rand() % (cloud->size() - 2);
		if (idx2 >= idx1) idx2++;
		if (idx3 >= idx1) idx3++;
		if (idx3 >= idx2) idx3++;
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		std::unordered_set<int> tempResult;
		//(y1-y2)x+(x2-x1)y+(x1*y2-x2*y1)=0
		float x1 = (*cloud)[idx1].x;
		float y1 = (*cloud)[idx1].y;
		float z1 = (*cloud)[idx1].z;
		float x2 = (*cloud)[idx2].x;
		float y2 = (*cloud)[idx2].y;
		float z2 = (*cloud)[idx2].z;
		float x3 = (*cloud)[idx3].x;
		float y3 = (*cloud)[idx3].y;
		float z3 = (*cloud)[idx3].z;
		float A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		float B = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		float C = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		float D = -(A*x1 + B * y1 + C * z1);
		if (A*A + B * B + C * C == 0) continue;
		for (int i = 0; i < cloud->size(); i++)
		{
			//Distance d = |Ax+By+Cz+D|/sqrt(A^2+B^2+C^2)
			float x = (*cloud)[i].x;
			float y = (*cloud)[i].y;
			float z = (*cloud)[i].z;
			float d = abs(A*x + B * y + C * z + D) / sqrt(A*A + B * B + C * C);
			if (d < distanceTol)
			{
				tempResult.insert(i);
			}
		}

		if (tempResult.size() > inliersResult.size())
		{
			inliersResult = tempResult;
		}
	}

	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;

}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(const std::vector<PointT>& points, std::vector<bool> &processed, KdTree<PointT>* tree, float distanceTol, int idx, std::vector<int> &cluster)
{
	processed[idx] = true;
	cluster.push_back(idx);
	std::vector<int> nearby = tree->search(points[idx], distanceTol);
	for (auto nearby_idx : nearby)
	{
		if (!processed[nearby_idx])
		{
			Proximity(points, processed, tree, distanceTol, nearby_idx, cluster);
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<PointT>& points, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed;
	processed.resize(points.size());
	for (int idx = 0; idx < points.size(); idx++)
	{
		if (!processed[idx])
		{
			std::vector<int> cluster;
			Proximity(points, processed, tree, distanceTol, idx, cluster);
			if (cluster.size() >= minSize && cluster.size() <= maxSize)
			{
				clusters.push_back(cluster);
			}
		}
	}
	return clusters;

}
template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	typename pcl::PointCloud<PointT>::Ptr croppedCloud1(new pcl::PointCloud<PointT>);
	pcl::CropBox<PointT> cropbox(true);
	cropbox.setInputCloud(cloud);
	cropbox.setMin(minPoint);
	cropbox.setMax(maxPoint);
	cropbox.filter(*croppedCloud1);

	// filter car
	typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>);
	cropbox.setInputCloud(croppedCloud1);
	cropbox.setMin(Eigen::Vector4f(-1.3, -1.5, -1, 1));
	cropbox.setMax(Eigen::Vector4f(2, 1.3, 0, 1));
	cropbox.setNegative(true);
	cropbox.filter(*croppedCloud);

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
	// Create the filtering object
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(croppedCloud);
	sor.setLeafSize(filterRes, filterRes, filterRes);
	sor.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

	return filteredCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
	//Create two new point clouds, one cloud with obstacles and other with segmented plane
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setNegative(false);
	extract.setIndices(inliers);
	typename pcl::PointCloud<PointT>::Ptr cloud_pos(new pcl::PointCloud<PointT>);
	extract.filter(*cloud_pos);
	extract.setNegative(true);
	typename pcl::PointCloud<PointT>::Ptr cloud_neg(new pcl::PointCloud<PointT>);
	extract.filter(*cloud_neg);
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_neg, cloud_pos);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
	auto res = RansacPlane(cloud, maxIterations, distanceThreshold);
	inliers->indices.resize(res.size());
	int idx = 0;
	for (auto it = res.begin(); it != res.end(); ++it, ++idx)
	{
		inliers->indices[idx] = *it;
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
	return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	// Creating the KdTree object for the search method of the extraction
	std::vector<PointT> cloud_as_vector(cloud->size());
	KdTree<PointT> tree;
	for (int idx = 0; idx < cloud->size(); idx++)
	{
		cloud_as_vector[idx] = cloud->at(idx);
		tree.insert(cloud_as_vector[idx], idx);
	}
	std::vector<std::vector<int> > cluster_indices = euclideanCluster(cloud_as_vector, &tree, clusterTolerance, minSize, maxSize);
	int j = 0;
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		//for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		for(auto pit = it->begin(); pit!=it->end();pit++)
			cloud_cluster->push_back((*cloud)[*pit]);
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		clusters.push_back(cloud_cluster);
		j++;
	}
	

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
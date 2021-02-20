/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

namespace quiz_ransac2d {


	pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		// Add inliers
		float scatter = 0.6;
		for (int i = -5; i < 5; i++)
		{
			double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
			double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
			pcl::PointXYZ point;
			point.x = i + scatter * rx;
			point.y = i + scatter * ry;
			point.z = 0;

			cloud->points.push_back(point);
		}
		// Add outliers
		int numOutliers = 10;
		while (numOutliers--)
		{
			double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
			double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
			pcl::PointXYZ point;
			point.x = 5 * rx;
			point.y = 5 * ry;
			point.z = 0;

			cloud->points.push_back(point);

		}
		cloud->width = cloud->points.size();
		cloud->height = 1;

		return cloud;

	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
	{
		ProcessPointClouds<pcl::PointXYZ> pointProcessor;
		return pointProcessor.loadPcd("src/sensors/data/pcd/simpleHighway.pcd");
	}


	pcl::visualization::PCLVisualizer::Ptr initScene()
	{
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->initCameraParameters();
		viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
		viewer->addCoordinateSystem(1.0);
		return viewer;
	}

	std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
	{
		std::unordered_set<int> inliersResult;
		srand(time(NULL));

		// For max iterations 
		for (; --maxIterations;)
		{
			// Randomly sample subset and fit line
			int idx1 = rand() % cloud->size();
			int idx2 = rand() % (cloud->size() - 1);
			if (idx2 >= idx1) idx2++;
			// Measure distance between every point and fitted line
			// If distance is smaller than threshold count it as inlier
			std::unordered_set<int> tempResult;
			//(y1-y2)x+(x2-x1)y+(x1*y2-x2*y1)=0
			float x1 = (*cloud)[idx1].x;
			float y1 = (*cloud)[idx1].y;
			float x2 = (*cloud)[idx2].x;
			float y2 = (*cloud)[idx2].y;
			float A = y1 - y2;
			float B = x2 - x1;
			float C = x1 * y2 - x2 * y1;
			if (A*A + B * B == 0) continue;
			for (int i = 0; i < cloud->size(); i++)
			{
				//Distance d = |Ax+By+C|/sqrt(A^2+B^2)
				float x = (*cloud)[i].x;
				float y = (*cloud)[i].y;
				float d = abs(A*x + B * y + C) / sqrt(A*A + B * B);
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

	std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
			float D = -(A*x1 + B*y1 + C*z1);
			if (A*A + B * B +C*C== 0) continue;
			for (int i = 0; i < cloud->size(); i++)
			{
				//Distance d = |Ax+By+Cz+D|/sqrt(A^2+B^2+C^2)
				float x = (*cloud)[i].x;
				float y = (*cloud)[i].y;
				float z = (*cloud)[i].z;
				float d = abs(A*x + B * y + C*z+D) / sqrt(A*A + B * B+C*C);
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


	int main()
	{

		// Create viewer
		pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

		// Create data
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


		// TODO: Change the max iteration and distance tolerance arguments for Ransac function
		std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

		pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

		for (int index = 0; index < cloud->points.size(); index++)
		{
			pcl::PointXYZ point = cloud->points[index];
			if (inliers.count(index))
				cloudInliers->points.push_back(point);
			else
				cloudOutliers->points.push_back(point);
		}


		// Render 2D point cloud with inliers and outliers
		if (inliers.size())
		{
			renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
			renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
		}
		else
		{
			renderPointCloud(viewer, cloud, "data");
		}

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
		}
		return 0;
	}
}

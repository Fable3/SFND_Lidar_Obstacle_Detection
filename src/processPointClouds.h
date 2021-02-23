// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"


template<typename PointT>
struct Node
{
	float point[3];
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
		: id(setId), left(NULL), right(NULL)
	{
		point[0] = arr.x;
		point[1] = arr.y;
		point[2] = arr.z;
	}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
		: root(NULL)
	{}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		int depth = 0;
		Node<PointT> **target_node = &root;
		for (;;)
		{
			int d = depth % 3;
			if (*target_node == NULL)
			{
				*target_node = new Node<PointT>(point, id);
				break;
			}
			else if (point.data[d] < (*target_node)->point[d])
			{
				target_node = &(*target_node)->left;
			}
			else
			{
				target_node = &(*target_node)->right;
			}
			depth++;
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT pnt, float distanceTol)
	{
		std::vector<int> ids;
		float target[3] = { pnt.x, pnt.y, pnt.z };
		searchHelper(ids, root, 0, target, distanceTol);
		return ids;
	}
	void searchHelper(std::vector<int> &ids, Node<PointT> *node, int depth, float target[3], float distanceTol)
	{
		if (node == NULL) return;
		int dim = depth % 3;
		float target_val = target[dim];
		float node_val = node->point[dim];
		if (target_val < node_val - distanceTol)
		{
			searchHelper(ids, node->left, depth + 1, target, distanceTol);
		}
		else if (target_val > node_val + distanceTol)
		{
			searchHelper(ids, node->right, depth + 1, target, distanceTol);
		}
		else
		{
			float dist_square = 0;
			for (int i = 0; i < 3; i++) dist_square += (target[i] - node->point[i])*(target[i] - node->point[i]);
			if (dist_square <= distanceTol * distanceTol)
			{
				ids.push_back(node->id);
			}
			searchHelper(ids, node->left, depth + 1, target, distanceTol);
			searchHelper(ids, node->right, depth + 1, target, distanceTol);
		}
	}
};


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

	std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
	void Proximity(const std::vector<PointT>& points, std::vector<bool> &processed, KdTree<PointT>* tree, float distanceTol, int idx, std::vector<int> &cluster);
	std::vector<std::vector<int>> euclideanCluster(const std::vector<PointT>& points, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */
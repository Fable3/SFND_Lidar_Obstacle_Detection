/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(const std::vector<float> &arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<unsigned int NDIM>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(const std::vector<float> &point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		int depth = 0;
		Node **target_node = &root;
		for (;;)
		{
			int d = depth % NDIM;
			if (*target_node == NULL)
			{
				*target_node = new Node(point, id);
				break;
			}
			else if (point[d] < (*target_node)->point[d])
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
	std::vector<int> search(const std::vector<float> &target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(ids, root, 0, target, distanceTol);
		return ids;
	}
	void searchHelper(std::vector<int> &ids, Node *node, int depth, const std::vector<float> &target, float distanceTol)
	{
		if (node == NULL) return;
		int dim = depth % NDIM;
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
			for (int i = 0; i < target.size(); i++) dist_square += (target[i] - node->point[i])*(target[i] - node->point[i]);
			if (dist_square <= distanceTol*distanceTol)
			{
				ids.push_back(node->id);
			}
			searchHelper(ids, node->left, depth + 1, target, distanceTol);
			searchHelper(ids, node->right, depth + 1, target, distanceTol);
		}
	}

	

};





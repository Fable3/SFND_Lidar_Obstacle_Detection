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
		insertHelper(point, id, 0, root);
	}
	void insertHelper(const std::vector<float> &point, int id, int depth, Node *&target_node)
	{
		int d = depth % 2;
		if (target_node == NULL)
		{
			target_node = new Node(point, id);
		}
		else if (point[d] < target_node->point[d])
		{
			insertHelper(point, id, depth + 1, target_node->left);
		}
		else
		{
			insertHelper(point, id, depth + 1, target_node->right);
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};





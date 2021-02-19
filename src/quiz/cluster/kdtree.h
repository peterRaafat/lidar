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

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insert(point, id, &root, 0);
	}

	void insert(std::vector<float> point, int id, struct Node** pp_node, int depth)
	{
		if (*pp_node == NULL)
		{
			*pp_node = new Node(point, id);
		}
		else
		{
			if ((depth % 2) == 0)
			{
				//split by x
				if (point[0] >= (*pp_node)->point[0])
				{
					//go right
					insert(point, id, &((*pp_node)->right), ++depth);
				}
				else
				{
					insert(point, id, &((*pp_node)->left), ++depth);
				}
			}
			else
			{
				//split by y
				if (point[1] >= (*pp_node)->point[1])
				{
					//go right
					insert(point, id, &((*pp_node)->right), ++depth);
				}
				else
				{
					insert(point, id, &((*pp_node)->left), ++depth);
				}
			}
				
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};





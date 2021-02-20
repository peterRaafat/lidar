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
		search(target, distanceTol, &root, 0, &ids);
		return ids;
	}

	void search(std::vector<float> target, float distanceTol, struct Node** pp_node ,int depth, std::vector<int>* p_result)
	{
		if (*(pp_node) != NULL)
		{
			if ((depth % 2) == 0)
			{
				//check by X
				if (abs((*(pp_node))->point[0] - target[0]) <= distanceTol)
				{
					//point is in the box
					float dist = sqrt((pow(((*(pp_node))->point[0] - target[0]), 2)) + (pow(((*(pp_node))->point[1] - target[1]), 2)));
					if (dist <= distanceTol)
					{
						//add point to the vector
						p_result->push_back((*(pp_node))->id);
					}
					search(target, distanceTol, &(*(pp_node))->left, ++depth, p_result);
					search(target, distanceTol, &(*(pp_node))->right, ++depth, p_result);
				}
				else
				{
					//point is outside the box we need to check whether we split left or right
					if ((*(pp_node))->point[0] > target[0])
					{
						//ignore the right section
						search(target, distanceTol, &(*(pp_node))->left, ++depth, p_result);
					}
					else
					{
						//ignore left section
						search(target, distanceTol, &(*(pp_node))->right, ++depth, p_result);
					}
				}
			}
			else
			{
				//check by Y
				if (abs((*(pp_node))->point[1] - target[1]) <= distanceTol)
				{
					//point is in the box
					float dist = sqrt((pow(((*(pp_node))->point[0] - target[0]), 2)) + (pow(((*(pp_node))->point[1] - target[1]), 2)));
					if (dist <= distanceTol)
					{
						//add point to the vector
						p_result->push_back((*(pp_node))->id);
					}
					search(target, distanceTol, &(*(pp_node))->left, ++depth, p_result);
					search(target, distanceTol, &(*(pp_node))->right, ++depth, p_result);
				}
				else
				{
					//point is outside the box we need to check whether we split left or right
					if ((*(pp_node))->point[1] > target[1])
					{
						//ignore the right section
						search(target, distanceTol, &(*(pp_node))->left, ++depth, p_result);
					}
					else
					{
						//ignore left section
						search(target, distanceTol, &(*(pp_node))->right, ++depth, p_result);
					}
				}
			}
		}
	}

};





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
			int cd = depth % 3;
			if (point[cd] < ((*pp_node)->point[cd]))
			{
				insert(point, id, &((*pp_node)->left), depth+1);
			}
			else
			{
				insert(point, id, &((*pp_node)->right), depth+1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(target, distanceTol, root, 0, &ids);
		//searchRecursiveHelper(ids, target, distanceTol, root, 0);
		return ids;
	}
	bool IsInBoundary(std::vector<float> pointA, std::vector<float> pointB, float distance)
	{
		bool result = true;
		result = (abs(pointA[0] - (pointB[0])) <= distance) && result;
		result = (abs(pointA[1] - (pointB[1])) <= distance) && result;
		result = (abs(pointA[2] - (pointB[2])) <= distance) && result;
		return result;
	}

	void search(std::vector<float> target, float distanceTol, struct Node* p_node ,int depth, std::vector<int>* p_result)
	{
		if (p_node != NULL)
		{
			if (IsInBoundary(p_node->point, target, distanceTol))
			{
				float x_2 = (p_node->point[0] - target[0]) * (p_node->point[0] - target[0]);
				float y_2 = (p_node->point[1] - target[1]) * (p_node->point[1] - target[1]);
				float z_2 = (p_node->point[2] - target[2]) * (p_node->point[2] - target[2]);
				float dist = sqrt(x_2 + y_2 + z_2);
				if (dist <= distanceTol)
				{
					p_result->push_back(p_node->id);
				}
			}
			int cd = depth % 3;
			if ((target[cd] - distanceTol) < p_node->point[cd])
			{
				search(target, distanceTol, p_node->left, depth+1, p_result);
			}

			if ((target[cd] + distanceTol) > p_node->point[cd])
			{
				search(target, distanceTol, p_node->right, depth+1, p_result);
			}

		}
	}

};





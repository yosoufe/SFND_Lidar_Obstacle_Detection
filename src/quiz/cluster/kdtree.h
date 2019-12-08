/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
			: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
			: root(NULL)
	{
	}

	void insert_recursive(Node *&node, std::vector<float> &point, int &id, size_t level)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
			return;
		}
		size_t index = level % 2;
		if (point[index] < node->point[index])
			insert_recursive(node->left, point, id, level+1 );
		else
			insert_recursive(node->right, point, id, level+1);
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insert_recursive(root, point, id, 0);
	}

	void search_recursive(std::vector<int> &buf,
												std::vector<float> &target,
												float distanceTol,
												Node *&node,
												size_t level)
	{
		if (node == NULL)
		{
			return;
		}
		size_t index = level % 2;

		// if point is in the box
		if (std::fabs(node->point[0] - target[0]) < distanceTol &&
				std::fabs(node->point[1] - target[1]) < distanceTol)
		{
			float dist = std::sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
														 (node->point[1] - target[1]) * (node->point[1] - target[1]));
			buf.push_back(node->id);
		}

		if (node->point[index] > (target[index] - distanceTol))
			search_recursive(buf, target, distanceTol, node->left, level + 1);

		if (node->point[index] < (target[index] + distanceTol))
			search_recursive(buf, target, distanceTol, node->right, level + 1);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids_buffer;

		search_recursive(ids_buffer, target, distanceTol, root, 0);

		return ids_buffer;
	}
};

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
		insertHelper(point, id, root, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, ids, root, 0);
		return ids;
	}
	
private:

	void insertHelper(const std::vector<float> & point, int id, Node *& node, size_t level)
	{
		if (!node)
		{
			node = new Node(point, id);
			return;
		}

		size_t i = level % point.size();
		if (point[i] <= node->point[i])
			insertHelper(point, id, node->left, level + 1);
		else
			insertHelper(point, id, node->right, level + 1);

	}

	float distanceSquared(const std::vector<float> & p1, const std::vector<float> & p2)
	{
		assert(p1.size() == p2.size());
		float d = 0;
		for (size_t i = 0; i < p1.size(); i++)
		{
			float delta = p1[i] - p2[i];
			d += delta * delta;
		}
		return d;
	}

	void searchHelper(const std::vector<float> & target, float distanceTol, std::vector<int> & nearbyPoints, Node * node, size_t level)
	{
		if (!node)
			return;

		size_t i = level % target.size();

		if ((target[i] + distanceTol) < node->point[i])
		{
			searchHelper(target, distanceTol, nearbyPoints, node->left, level + 1);
		}
		else if ((target[i] - distanceTol) > node->point[i])
		{
			searchHelper(target, distanceTol, nearbyPoints, node->right, level + 1);
		}
		else
		{
			float distSquared = distanceSquared(target, node->point);
			if (distSquared <= (distanceTol * distanceTol))
				nearbyPoints.push_back(node->id);

			searchHelper(target, distanceTol, nearbyPoints, node->left, level + 1);
			searchHelper(target, distanceTol, nearbyPoints, node->right, level + 1);
		}
	}
};





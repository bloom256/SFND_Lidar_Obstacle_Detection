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
		inseertHelper(point, id, true, root);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::cout << "kd-tree serach: [" << target[0] << "," << target[1] << "] " << " dist " << distanceTol << std::endl;
		std::vector<int> ids;
		searchHelper(target, distanceTol, ids, true, root);
		std::cout << "kd-tree serach: " << ids.size() << " points found" << std::endl;
		return ids;
	}
	
private:

	void inseertHelper(std::vector<float> point, int id, bool xSplit, Node *& node)
	{
		if (!node)
		{
			node = new Node(point, id);
			return;
		}

		if (xSplit)
		{
			if (point[0] <= node->point[0])
				inseertHelper(point, id, !xSplit, node->left);
			else
				inseertHelper(point, id, !xSplit, node->right);
		}
		else
		{
			if (point[1] <= node->point[1])
				inseertHelper(point, id, !xSplit, node->left);
			else
				inseertHelper(point, id, !xSplit, node->right);
		}
	}

	void searchHelper(std::vector<float> target, float distanceTol, std::vector<int> & nearbyPoints, bool xSplit, Node * node)
	{
		if (!node)
			return;

		cout << "searchHelper: node " << node->id << " [" << node->point[0] << "," << node->point[1] << "]" << " xSplit " << xSplit << std::endl;

		if (xSplit)
		{
			if ((target[0] + distanceTol) < node->point[0])
			{
				//std::cout << "1" << std::endl;
				searchHelper(target, distanceTol, nearbyPoints, !xSplit, node->left);
			}
			else if ((target[0] - distanceTol) > node->point[0])
			{
				//std::cout << "2" << std::endl;
				searchHelper(target, distanceTol, nearbyPoints, !xSplit, node->right);
			}
			else
			{
				//std::cout << "3" << std::endl;
				float dist = (Eigen::Vector3f(target[0], target[1], 0) - Eigen::Vector3f(node->point[0], node->point[1], 0)).norm();
				if (dist < distanceTol)
				{
					std::cout << "hit" << std::endl;
					nearbyPoints.push_back(node->id);
				}

				searchHelper(target, distanceTol, nearbyPoints, !xSplit, node->left);
				searchHelper(target, distanceTol, nearbyPoints, !xSplit, node->right);
			}
		}
		else
		{
			if ((target[1] + distanceTol) < node->point[1])
			{
				//std::cout << "4" << std::endl;
				searchHelper(target, distanceTol, nearbyPoints, !xSplit, node->left);
			}
			else if ((target[1] - distanceTol) > node->point[1])
			{
				//std::cout << "5" << std::endl;
				searchHelper(target, distanceTol, nearbyPoints, !xSplit, node->right);
			}
			else
			{
				//std::cout << "6" << std::endl;
				float dist = (Eigen::Vector3f(target[0], target[1], 0) - Eigen::Vector3f(node->point[0], node->point[1], 0)).norm();
				if (dist < distanceTol)
				{
					std::cout << "hit" << std::endl;
					nearbyPoints.push_back(node->id);
				}

				searchHelper(target, distanceTol, nearbyPoints, !xSplit, node->left);
				searchHelper(target, distanceTol, nearbyPoints, !xSplit, node->right);
			}
		}
	}
};





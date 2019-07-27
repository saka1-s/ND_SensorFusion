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

void insertHelper(Node **node, std::vector<float> point, int id,int depth)
{

	if(*node == NULL)
	{
		*node = new Node(point, id);
		return;
	}
	bool left;
	if (depth%2 == 0) {
		// compare x
		left = point[0] < (*node)->point[0];
	} else {
		// comapre y
		left = point[1] < (*node)->point[1];
	}
	if(left)
	{
		insertHelper(&(*node)->left, point, id, depth+1);
	}
	else
	{
		insertHelper(&(*node)->right, point, id, depth+1);
	}
}

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
		insertHelper(&root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};





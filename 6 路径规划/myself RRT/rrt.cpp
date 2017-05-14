
/**
	@RRT空间下的构造函数定义(RRT算法用到的功能函数)
	@author zhao lu
	@version 20170415
*/

#include <path_planning/rrt.h>    //rrt.h在path_planning包里
#include <math.h>
#include <cstddeef>
#include <iostream>

using namespace rrt;

/************************** Function Definitions *****************************/

/**
	@初始化class类， initializes source to 0,0，adds sorce to rrtTree
	@author zhao lu
	@version 20170415
*/
RRT::RRT()
{
	RRT::rrtNode newNode;
	newNode.posX = 0;
	newNode.posY = 0;
	newNode.parentID = 0;
	newNode.nodeID = 0;
	rrtTree.push_back(newNode);   //在尾部加入一个新的节点
}

/**
	 @初始化起始节点的坐标 ，adds sorce to rrtTree
	 @author zhao lu
	 @version 20170415
*/
RRT::RRT(double input_PosX, double input_PosY)
{
	RRT::rrtNode newNode;
	newNode.posX = input_PosX;
	newNode.posY = input_PosY;
	newNode.parentID = 0;
	newNode.nodeID = 0;
	rrtTree.push_back(newNode);
}

/**
	@返回当前的RRT树
	@author zhao lu
	@version 20170415
*/
vector<RRT::rrtNode> RRT::getTree()
{
	return rrtTree;
}

/**
	 @For setting the rrtTree to the inputTree
	 @param rrtTree
*/
void RRT::setTree(vector<RRT::rrtNode> input_rrtTree)
{
	rrtTree = input_rrtTree;
}

/**
	@to get the number of nodes in the rrt Tree
	@return tree size
*/
int RRT::getTreeSize()
{
	return rrtTree.size();
}

/**
	@adding a new node to the rrt Tree

*/
void RRT::addNewNode(RRT::rrtNode node)
{
	rrtTree.push_back(node);
}

/**
	@removing a node from the RRT Tree
	@return the removed tree
*/
RRT::rrtNode RRT::removeNode(int id)   //rrtNode是基类，removeNode派生类
{
	RRT::rrtNode tempNode = rrtTree[id];
	rrtTree.erase(rrtTree.begin() + id );   //销毁内存
	return tempNode;
}

/**
	 getting a specific node
	 @param node id for the required node
	 @return node in the rrtNode structure
*/
RRT::rrtNode RRT::getNode(inr id)
{
	return rrtTree[id];
}

/**
	 在已给出的树节点中得到最近的节点
	 @param X position in X cordinate
	 @param Y position in Y cordinate
	 @return nodeID of the nearest Node
*/
int RRT::getNearesNodeID(double X, double Y)
{
	int i, returnID;
	double distance = 9999, tempDistance;  //WHY9999
	for (i = 0; i < this->getTreeSize(); i++)
	{
		tempDistance = getEuclideanDistanc((X, Y, getPosX(i), getPosY(i));  //得到欧几里德距离
		if (tempDistance < distance)
		{
			distance = tempDistance;
			returnID = id;
		}
	}
	return returnID;
}

/**
	returns X coordinate(坐标) of the given node
*/
double RRT::getPosX(int nodeID)
{
	return rrtTree[nodeID].posX;
}

/**
	returns Y coordinate(坐标) of the given node
*/
double RRT::getPosY(int nodeID)
{
	return rrtTree[nodeID].posY;
}

/**
	 set X coordinate of the given node
*/
void RRT::setPosX(int nodeID, double input_PosX)
{
	rrtTree[nodeID].posX = input_PosX;
}

/**
  set Y coordinate of the given node
*/
void RRT::setPosY(int nodeID, double input_PosY)
{
	rrtTree[nodeID].posY = input_PosY;
}

/**
	 returns parentID of the given node
*/
RRT::rrtNode RRT::getParent(int id)
{
	return rrtTree[rrtTree[id].parentID];
}

/**
   set parentID of the given node
*/
void RRT::setParentID(int nodeID, int parentID)
{
	rrtTree[nodeID].parentID = parentID;
}

/**
   add a new childID to the children list of the given node
*/
void RRT::addChildID(int nodeID, int childID)
{
	rrtTree[nodeID].children.push_back(childID);
}

/**
  returns the children list of the given node
*/
vector<int> RRT::getChildren(int id)
{
	return rrtTree[id].children;
}

/**
  returns the children list of the given node
*/
vector<int> RRT::getChildren(int id)
{
	return rrtTree[id].children;
}

/**
   returns number of children of a given node
*/
int RRT::getChildrenSize(int nodeID)
{
	return rrtTree[nodeID].children.size();
}

/**
  返回euclidean距离由俩个已经有的X,Y的坐标
*/
double RRT::getEuclideanDistance(double sourceX, double sourceY, double destinationX, double destinationY)
{
	return sqrt(pow(destinationX - sourceX, 2) + pow(destinationY - sourceY, 2));
}

/**
	@回溯路径，得到近似最优路径
	@param endNodeID of the end node
	@返回得到路径中每个节点的ID
*/
vector<int> RRT::getRootToEndPath(int endNodeID)
{
	vector<int> path;
	path.push_back(endNodeID);   //将最后一个节点放入尾部
	while (rrtTree[path.front()].nodeID != 0)  //当起始节点的ID不为0，则一直插入
	{
		//在指定位置begin位置插入，front()返回元素值，begin()返回迭代器（指针），指针指向的值=front()返回元素值
		//一直在最前面插入，直到=0
		path.insert(path.begin(), rrtTree[path.front()].parentID);
	}
	return path;   //返回一个动态数组，因为vector是一种数据结构（类）
}

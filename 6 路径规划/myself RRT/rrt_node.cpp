
/**
	@RRT算法实现
	@author zhao lu
	@version 20170415
*/

#include <ros/ros.h>
//rrt算法有关头文件
#include <visualization_msgs/Marker.h>   //数据可视化，在RVIZ中
#include <geometry_msgs/Point.h>   //ros中点的信息，x,y,z
#include <path_planning/rrt.h>      //关于rrt相关
#include <path_planning/obstacles.h>    //障碍物
//C++中的库
#include <iostream>
#include <cmath.h>  //和math里的函数用法一样
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>

#define success false     //????????
#define running true      //可能是能运行才算成功

using namespace rrt; 

bool status = running;     //状态

//初始化路标
void initializeMarkers(visualization_msgs::Marker &sourcePoint,   //可能是起始点
	visualization_msgs::Marker &goalPoint,   //目标点
	visualization_msgs::Marker &randomPoint,   //随机找到的点
	visualization_msgs::Marker &rrtTreeMarker,  //
	visualization_msgs::Marker &finalPath)
{
	//初始化节点
	sourcePoint.header.frame_id = goalPoint.header.frame_id = randomPoint.header.frame_id = rrtTreeMarker.header.frame_id = finalPath.header.frame_id = "path_planner";
	sourcePoint.header.stamp = goalPoint.header.stamp = randomPoint.header.stamp = rrtTreeMarker.header.stamp = finalPath.header.stamp = ros::Time::now();
	sourcePoint.ns = goalPoint.ns = randomPoint.ns = rrtTreeMarker.ns = finalPath.ns = "path_planner";
	sourcePoint.action = goalPoint.action = randomPoint.action = rrtTreeMarker.action = finalPath.action = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = finalPath.pose.orientation.w = 1.0;

	//为每一种路标设置ID号
	sourcePoint.id = 0;
	goalPoint.id = 1;
	randomPoint.id = 2;
	rrtTreeMarker.id = 3;
	finalPath.id = 4;

	//设置尺度
	rrtTreeMarker.scale.x = 0.2;
	finalPath.scale.x = 1;
	sourcePoint.scale.x = goalPoint.scale.x = randomPoint.scale.x = 2;
	sourcePoint.scale.y = goalPoint.scale.y = randomPoint.scale.y = 2;
	sourcePoint.scale.z = goalPoint.scale.z = randomPoint.scale.z = 1;

	//设置颜色
	sourcePoint.color.r = 1.0f;
	goalPoint.color.g = 1.0f;
	randomPoint.color.b = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

	finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;

	sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = finalPath.color.a = 1.0f;
}

//障碍物
vector<vector<geometry_msgs::Point> >  getObstacles()
{
	obstacles obst;
	return obst.getObsrtaclesArry();
}

//添加分支到树结构
void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode, RRT &myRRT)
{
	geometry_msgs::Point point;
	point.x = tempNode.posX;
	point.y = tempNode.posY;
	point.z = 0;
	rrtTreeMarker.points.push_back(point);    //在树的尾部加入节点

	RRT::rrtNode parentNode = myRRT.getParent(tempNode.nodeID);

	point.x = parentNode.posX;    //把上一个节点当做父节点
	point.y = parentNode.posY;
	point.z = 0;

	rrtTreeMarker.points.push_back(point);  //把此节点当做尾节点
}

//检查节点是否在边界内
bool checkIfInsideBoundary(RRT::rrtNode &tempNode)
{
	if (tempNode.posX < 0 || tempNode.posY < 0 || tempNode.posX > 100 || tempNode.posY > 100)
		return false;
	else
		return true;
}

//检查障碍物是否在边界外
bool checkIfOutsideObstacles(vector<vector<geometry_msgs::Point> > &obstArray, RRT::rrtNode &tempNode)
{
	double AB, AD, AMAB, AMAD;

	for (int i = 0; i<obstArray.size(); i++)
	{
		//pow计算x的y次幂
		AB = (pow(obstArray[i][0].x - obstArray[i][1].x, 2) + pow(obstArray[i][0].y - obstArray[i][1].y, 2));
		AD = (pow(obstArray[i][0].x - obstArray[i][3].x, 2) + pow(obstArray[i][0].y - obstArray[i][3].y, 2));
		AMAB = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][1].x - obstArray[i][0].x)) + ((tempNode.posY - obstArray[i][0].y) * (obstArray[i][1].y - obstArray[i][0].y)));
		AMAD = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][3].x - obstArray[i][0].x)) + ((tempNode.posY - obstArray[i][0].y) * (obstArray[i][3].y - obstArray[i][0].y)));
		//(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
		if ((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
		{
			return false;
		}
	}
	return true;
}

//产生随机节点
void generateTempPoint(RRT::rrtNode &tempNode)
{
	int x = rand() % 150 + 1;
	int y = rand() % 150 + 1;
	tempNode.posX = x;
	tempNode.posY = y;
}

//增加新的节点到RRT树
//param :树结构，树节点，步长，障碍物
bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, int rrtStepSize, vector< vector<geometry_msgs::Point> > &obstArray)
{
	int nearestNodeID = myRRT.getNearestNodeID(tempNode.posX, tempNode.posY);  //最近节点ID

	RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID); //最近树节点

	double theta = atan2(tempNode.posY - nearestNode.posY, tempNode.posX - nearestNode.posX);

	tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));
	tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));

	if (checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(obstArray, tempNode))
	{
		tempNode.parentID = nearestNodeID;
		tempNode.nodeID = myRRT.getTreeSize();
		myRRT.addNewNode(tempNode);
		return true;
	}
	else
		return false;
}

//查看节点之间的距离
bool checkNodetoGoal(int X, int Y, RRT::rrtNode &tempNode)
{
	double distance = sqrt(pow(X - tempNode.posX, 2) + pow(Y - tempNode.posY, 2));
	if (distance < 3)
	{
		return true;
	}
	return false;
}

//得到最终路径的数据
void setFinalPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, visualization_msgs::Marker &finalpath, int goalX, int goalY)
{
	RRT::rrtNode tempNode;
	geometry_msgs::Point point;
	for (int j = 0; j<rrtPaths[i].size(); j++)
	{
		tempNode = myRRT.getNode(rrtPaths[i][j]);

		point.x = tempNode.posX;
		point.y = tempNode.posY;
		point.z = 0;

		finalpath.points.push_back(point);
	}

	point.x = goalX;
	point.y = goalY;
	finalpath.points.push_back(point);
}

int main(int argc, char** argv)
{
	//初始化ROS
	ros::init(argc,argv,"rrt_node");   //节点名称rrt_node
	ros::NodeHandle n;

	//定义发布者
	ros::Publisher rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);  //发布消息


	//定义要可视化的路标
	visualization_msgs::Marker sourcePoint;
	visualization_msgs::Marker goalPoint;
	visualization_msgs::Marker randomPoint;
	visualization_msgs::Marker rrtTreeMarker;
	visualization_msgs::Marker finalPath;

	initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, finalPath);

	//（1）设置起始点和目标点的坐标
	sourcePoint.pose.position.x = 2;
	sourcePoint.pose.position.y = 2;

	goalPoint.pose.position.x = 95;
	goalPoint.pose.position.y = 95;

	rrt_publisher.publish(sourcePoint);
	rrt_publisher.publish(goalPoint);
	ros::spinOnce();
	ros::Duration(0.01).sleep();

	srand(time(NULL));   //随机数发生器的初始化函数

	//（2）初始化节点？？？？
	RRT myRRT(2.0, 2.0);   //这个函数没找到？？？？？
	int goalX, goalY;
	goalX = goalY = 95;

	int rrtStepSize = 3;

	vector< vector<int> > rrtPaths;
	vector<int> path;
	int rrtPathLimit = 1;   //只能有一条路径

	int shortestPathLength = 9999;    //最短测试路径？？？？？？？
	int shortestPath = -1;

	RRT::rrtNode tempNode;

	vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles();  //得到障碍物的形式

	bool addNodeResult = false, nodeToGoal = false;

	while (ros::ok() && status)
	{
		if (rrtPaths.size() < rrtPathLimit)
		{
			generateTempPoint(tempNode);
			//std::cout<<"tempnode generated"<<endl;
			addNodeResult = addNewPointtoRRT(myRRT, tempNode, rrtStepSize, obstacleList);
			if (addNodeResult)
			{
				// std::cout<<"tempnode accepted"<<endl;
				addBranchtoRRTTree(rrtTreeMarker, tempNode, myRRT);
				// std::cout<<"tempnode printed"<<endl;
				nodeToGoal = checkNodetoGoal(goalX, goalY, tempNode);
				if (nodeToGoal)
				{
					path = myRRT.getRootToEndPath(tempNode.nodeID);
					rrtPaths.push_back(path);
					std::cout << "New Path Found. Total paths " << rrtPaths.size() << endl;
					//ros::Duration(10).sleep();
					//std::cout<<"got Root Path"<<endl;
				}
			}
		}

		else //if(rrtPaths.size() >= rrtPathLimit)
		{
			status = success;
			std::cout << "Finding Optimal Path" << endl;
			for (int i = 0; i<rrtPaths.size(); i++)
			{
				if (rrtPaths[i].size() < shortestPath)
				{
					shortestPath = i;
					shortestPathLength = rrtPaths[i].size();
				}
			}
			setFinalPathData(rrtPaths, myRRT, shortestPath, finalPath, goalX, goalY);
			rrt_publisher.publish(finalPath);
		}
		rrt_publisher.publish(sourcePoint);
		rrt_publisher.publish(goalPoint);
		rrt_publisher.publish(rrtTreeMarker);
		//rrt_publisher.publish(finalPath);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}

	return 1;
}




/*自适应尺度（多尺度）的核相关滤波*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "kcftracker.hpp"

using namespace cv;
using namespace std;
static const std::string RGB_WINDOW = "RGB Image window";
//static const std::string DEPTH_WINDOW = "DEPTH Image window";

#define Max_linear_speed 1
#define Min_linear_speed 0.4
#define Min_distance 1.0
#define Max_distance 5.0
#define Max_rotation_speed 0.75

float linear_speed = 0;
float rotation_speed = 0;

float k_linear_speed = (Max_linear_speed - Min_linear_speed) / (Max_distance - Min_distance);
float h_linear_speed = Min_linear_speed - k_linear_speed * Min_distance;

float k_rotation_speed = 0.004;
float h_rotation_speed_left = 1.2;
float h_rotation_speed_right = 1.36;
float distance_scale = 1.0;
int ERROR_OFFSET_X_left1 = 100;
int ERROR_OFFSET_X_left2 = 300;
int ERROR_OFFSET_X_right1 = 340;
int ERROR_OFFSET_X_right2 = 600;
int roi_height = 100;
int roi_width = 100;
cv::Mat rgbimage;
cv::Mat depthimage;
cv::Rect selectRect;
cv::Point origin;
cv::Rect result;

bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool enable_get_depth = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;
int DescriptorDim;
bool has_dectect_people;
// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
vector<float> myDetector;
float dist_val[5];

void onMouse(int event, int x, int y, int, void*)
{
	if (select_flag)
	{
		selectRect.x = MIN(origin.x, x);
		selectRect.y = MIN(origin.y, y);
		selectRect.width = abs(x - origin.x);
		selectRect.height = abs(y - origin.y);
		selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		bBeginKCF = false;
		select_flag = true;
		origin = cv::Point(x, y);
		selectRect = cv::Rect(x, y, 0, 0);   //目标初始框 cv::Rect=&roi，引用
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		select_flag = false;
		bRenewROI = true;
	}
}



class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber depth_sub_;
	HOGDescriptor hog;

public:
	ros::Publisher pub;

	ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &ImageConverter::imageCb, this);
		depth_sub_ = it_.subscribe("/camera/depth/image", 1, &ImageConverter::depthCb, this);
		pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/cmd_vel", 1000);
		
		cv::namedWindow(RGB_WINDOW);
		//cv::namedWindow(DEPTH_WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(RGB_WINDOW);
		//cv::destroyWindow(DEPTH_WINDOW);
	}



	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		//ros和opencv数据转换
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv_ptr->image.copyTo(rgbimage);
		
		
		cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

		if (bRenewROI)
		{
			
			tracker.init(selectRect, rgbimage);
			bBeginKCF = true;
			bRenewROI = false;
			enable_get_depth = false;
		}
		if (bBeginKCF)
		{
			result = tracker.update(rgbimage);
			cv::rectangle(rgbimage, result, cv::Scalar(0, 255, 0), 1, 8);
			enable_get_depth = true;
		}
		else
			cv::rectangle(rgbimage, selectRect, cv::Scalar(0, 255, 0), 2, 8, 0);

		cv::imshow(RGB_WINDOW, rgbimage);
		cv::waitKey(1);
	}
	
	
	void depthCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
			cv_ptr->image.copyTo(depthimage);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
		}

		if (enable_get_depth)
		{
			dist_val[0] = depthimage.at<float>(result.y + result.height / 3, result.x + result.width / 3);
			dist_val[1] = depthimage.at<float>(result.y + result.height / 3, result.x + 2 * result.width / 3);
			dist_val[2] = depthimage.at<float>(result.y + 2 * result.height / 3, result.x + result.width / 3);
			dist_val[3] = depthimage.at<float>(result.y + 2 * result.height / 3, result.x + 2 * result.width / 3);
			dist_val[4] = depthimage.at<float>(result.y + result.height / 2, result.x + result.width / 2);

			float distance = 0;
			int num_depth_points = 5;
			for (int i = 0; i < 5; i++)
			{
				if (dist_val[i] > 0.4 && dist_val[i] < 10.0)
					distance += dist_val[i];
				else
					num_depth_points--;
			}
			distance /= num_depth_points;

			//calculate linear speed 计算要发布给小车的线速度
			if (distance > Min_distance)   //前进
				linear_speed = distance * k_linear_speed + h_linear_speed;
			else if (distance <= Min_distance - 0.5)  //后退
			{
				
				linear_speed = -1 * ((Min_distance - 0.5) * k_linear_speed + h_linear_speed);
			}
			else
			{
				linear_speed = 0;
			}

			if (fabs(linear_speed) > Max_linear_speed)
				linear_speed = Max_linear_speed;

			//calculate rotation speed计算角速度
			int center_x = result.x + result.width / 2;
			if (center_x < ERROR_OFFSET_X_left1){
				printf("center_x <<<<<<<< ERROR_OFFSET_X_left1\n");
				rotation_speed = Max_rotation_speed / 5;
				//has_dectect_people = false;
				enable_get_depth = false;
				select_flag = false;
				bBeginKCF = false;
			}
			else if (center_x > ERROR_OFFSET_X_left1 && center_x < ERROR_OFFSET_X_left2)
				rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_left;
			else if (center_x > ERROR_OFFSET_X_right1 && center_x < ERROR_OFFSET_X_right2)
				rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_right;
			else if (center_x > ERROR_OFFSET_X_right2){
				printf("center_x >>>>>>>> ERROR_OFFSET_X_right2\n");
				rotation_speed = -Max_rotation_speed / 5;
				has_dectect_people = false;
				enable_get_depth = false;
				select_flag = false;
				bBeginKCF = false;
			}
			else
				rotation_speed = 0;

		}

		
		cv::waitKey(1);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kcf_tracker");
	ImageConverter ic;

	while (ros::ok())
	{
		ros::spinOnce();

		geometry_msgs::Twist twist;
		twist.linear.x = linear_speed;
		twist.linear.y = 0;
		twist.linear.z = 0;
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = rotation_speed;
		ic.pub.publish(twist);
		if (cvWaitKey(33) == 'q')
			break;
	}

	return 0;
}
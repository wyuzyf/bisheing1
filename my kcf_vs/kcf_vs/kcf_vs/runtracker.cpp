
/*基于fhog特征的目标追踪算法，VS2013运行环境*/


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <iostream>

#include "kcftracker.h"

#define VideoID 0

cv::Mat capframe;
//const char *windowname = "tracker";
bool select_flag = false;
cv::Rect select;
cv::Point origin;
bool bRenewROI = false; // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;

 void  onMouse(int event, int x, int y, void*)
{
	//Point origin;
	if (select_flag)
	{
		select.x = MIN(origin.x, x);
		select.y = MIN(origin.y, y);
		select.width = abs(x - origin.x);
		select.height = abs(y - origin.y);
		select &= cv::Rect(0, 0, capframe.cols, capframe.rows);
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		bBeginKCF = false;  // stop the KCF and display the ROI rectangle user chosed with the mouse
		select_flag = true;
		origin = cv::Point(x, y);
		select = cv::Rect(x, y, 0, 0);
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		select_flag = false;
		bRenewROI = true;
	}
}

int main(int argc, char *argv[])
{
	cv::VideoCapture cap(VideoID);

	const char *windowname = "KCF_Tracking";
	bool stop = false;

	bool HOG = true;
	bool FIXEDWINDOW = false;   //不是固定窗口
	bool MULTISCALE = true;   //使用多尺度
	bool LAB = false;  //不使用LAB颜色空间

	KCFTracker tracker(HOG,FIXEDWINDOW,MULTISCALE,LAB);

	if (!cap.isOpened())
	{
		std::cout << "Can not open camera." << std::endl;
	}

	// window for displaying the video stream 
	cv::namedWindow(windowname, 1);
	// capture for the mouse movement
	cv::setMouseCallback(windowname, (cv::MouseCallback)onMouse, 0);

	cv::Rect KCFResult;

	while (!stop)
	{
		cap >> capframe;

		if (bRenewROI)
		{
			KCFResult = select;
			tracker.init(KCFResult, capframe);
			bBeginKCF = true;

			/*
			KCFResult = tracker.update(capframe);
			std::cout << "width: " << KCFResult.width << "  height: " << KCFResult.height << std::endl;
			cv::rectangle(capframe, KCFResult, cv::Scalar(0, 0, 255), 2, 8, 0);
			*/
			bRenewROI = false;
		}


		if (bBeginKCF)
		{
			//cv::setMouseCallback(windowname, onMouse, 0);

			KCFResult = tracker.update(capframe);
			std::cout << "width: " << KCFResult.width << "  height: " << KCFResult.height << std::endl;
			cv::rectangle(capframe, KCFResult, cv::Scalar(0, 0, 255), 2, 8, 0);
		}


		else
			cv::rectangle(capframe, select, cv::Scalar(255, 0, 0), 2, 8, 0);

		imshow(windowname, capframe);


		if (cv::waitKey(1) == 27)  // ESC 27    不断刷新图像
			stop = true;

		std::cout << "x=" << select.x << " y=" << select.y << std::endl;

	}

	return 1;
}



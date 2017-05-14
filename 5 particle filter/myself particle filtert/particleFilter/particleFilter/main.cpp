
/************************************************************************/
/*
Description:	基本的粒子滤波目标跟踪（HSV颜色特征的直方图模型）
Author:			Zhao Lu
Email:			wyuzyf@126.com
Version:		2017-4-10
History:
Development environment:   VS2013+opencv3.10
*/
/************************************************************************/

#include <iostream >    //标准IO
#include <string>      //字符串
#include <iomanip>    //浮点打印输出
#include <sstream>    //字符串到数字的转换

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pf.h"

using namespace std;
using namespace cv;


int nFrameNum;          //全局变量默认初始化为0，局部变量必须初始化，因为局部变量存放在堆中
						//帧的数目

bool bSelectObject = false;     //区域选择标志
bool bTracking = false;       //开始跟踪标志

Point origin;       //鼠标按下时点的位置有可能和实际选取的点的位置不一样
extern Rect selection;     //感兴趣的区域大小,定义一个左上角点坐标为(x, y)的width*height矩形窗口
						//矩形的大小就是所选区域的大小

//PARTICLE particle[Particle_num];     //粒子
//const int Particle_num = 25;       //粒子个数

extern Mat Image;        //选取的目标图像

bool paused = false;    //暂停键

//鼠标响应函数，得到选择的区域，保存在selection
void onMouse(int event, int x, int y, int, void*)
{
	if (bSelectObject)    //判断有无选取目标区域
	{
		selection.x = MIN(x,origin.x);  //选最小的也就是让范围尽可能大
		selection.y = MIN(y,origin.y);
		selection.width = std::abs(x - origin.x);    //x轴是宽度，求绝对值
		selection.height = std::abs(y - origin.y);

		selection &= Rect(0, 0, Image.cols, Image.rows);  //&=赋值运算符，自右至左
	}

	switch (event)
	{
	case CV_EVENT_FLAG_LBUTTON:   //鼠标消息类型 左键点击
		origin = Point(x,y);   //选取的点的位置
		selection = Rect(x,y,0,0);  //选取目标的大小
		bSelectObject = true;   //选取成功
		bTracking = false;    //还没开始跟踪目标
		break;
	case CV_EVENT_LBUTTONUP:   //左键放开
		bSelectObject = false;     //不是选取目标的动作
		bTracking = true;        //开始跟踪目标
		nFrameNum = 0;    //目标帧是0，即还没正式开始
		break;
	}

}

void pause(void)
{
	int delay = 10;   //控制播放速度
	char c;   //键值
	c = (char)waitKey(delay);

	switch (c)
	{
	case 27:
		break;
	case 'p':
		paused = !paused;
		break;
	default:
		;
	}
}

int main(int argc, char *argv[])
{
	

	VideoCapture captRefrnc("F:\singlechip\a\bisheing\myself particle filtert\particleFilter\soccer");    //视频文件

	if (!captRefrnc.isOpened())
	{
		return -1;
	}

	const char* WIN_RESULT = "Result";     //窗口名称
	namedWindow(WIN_RESULT,CV_WINDOW_AUTOSIZE);    //大小可变

	setMouseCallback(WIN_RESULT,onMouse,0);     //鼠标响应函数
								//选取的窗口是WIN_RESULT，鼠标回调函数是onMouse

	Mat frame;   //视频的每一帧图像

	//bool paused = false;    //暂停键
	//PARTICLE *pParticles = particle;
	//PARTICLE *pParticles = new PARTICLE[sizeof(PARTICLE)* Particle_num];

	while (true)    //重复捕获图片
	{
		if (!paused)
		{
			captRefrnc >> frame;      //遍历每一帧图片
			if (frame.empty())
				break;
		}

		frame.copyTo(Image);   //采集一幅图像到缓存Image

		//选择目标后跟踪
		if (bTracking == true)
		{
			if (!paused)
			{
				nFrameNum++;  //帧数+1

				//进行粒子滤波
				particle_filter();
				
			}
		}
		imshow(WIN_RESULT, Image);
		pause();
	}
}
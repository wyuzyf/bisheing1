
#ifndef __PF_H
#define __PF_H

//#include <iostream >    //标准IO
//#include <string>      //字符串
//#include <iomanip>    //浮点打印输出
//#include <sstream>    //字符串到数字的转换

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//using namespace cv;

/*以下参数对结果有很大的影响，可以适时调整*/
 const int Particle_num = 25;       //粒子个数

//粒子放入的相关区域
const double A1 = 2.0;
const double A2 = -1.0;
const double B0 = 1.0;

//高斯随机数sigma参数
const double SIGMA_X = 1.0;
const double SIGMA_Y = 0.5;
const double SIGMA_SCALE = 0.001;

extern int nFrameNum;

//粒子结构体
typedef struct particle{
	
	//原始（选定）的，当前的，预测的
	double x;    //当前x坐标
	double y;    //当前y坐标
	double scale;      //窗口比例系数
	
	double xPre;      //x预测坐标位置
	double yPre;      //y预测坐标位置
	double scalePre;   //窗口预测比例系数
	
	double xOri;       //原始x坐标
	double yOri;       //原始y坐标
	Rect rect;      //原始区域大小
	MatND hist;      //粒子区域的特征直方图
	double weight;     //粒子的权重
}PARTICLE;
 PARTICLE particles[Particle_num];     //粒子

//直方图相关参数，特征的选取也会对结果影响巨大
// Quantize the hue to 30 levels色调
// and the saturation to 32 levels 饱和度
// value to 10 levels亮度值
int hbins = 180, sbins = 256, vbin = 10;
int histSize[] = { hbins, sbins, vbin };

//hue varies from 0 to 170,see cvtColor
float hranges[] = { 0, 180 };
//saturation varies from 0 (black-gray-white) to 255 (pure spectrum color)
float sranges[] = { 0, 256 };
//value varies from 0 (black-gray-white) to 255 (pure spectrum color)
float vranges[] = { 0, 256 };

const float *ranges[] = { hranges, sranges, vranges };
//we compute the histogram from the 0-th and 1-st channels
int channels[] = { 0, 1, 2 };


//功能函数
int particle_cmp(const void *p1, const void *p2);
void particle_filter(void);






#endif



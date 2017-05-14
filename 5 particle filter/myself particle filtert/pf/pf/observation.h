
/*@file
	有关目标模型的定义
   @author zhao lu
   @version 20170413
*/

#ifndef OBSERVATION_H
#define OBSERVATION_H

/**********************************Definitions******************************/

#include "defs.h"
#include "utils.h"

//将HSV分成若干的颜色区间，bin越多分辨率越高
#define NH 10
#define NS 10
#define NV 10

//HSV的最大值
#define H_MAX 360.0
#define S_MAX 1.0
#define V_MAX 1.0

//直方图中饱和度和亮度值的最低阈值
#define S_THRESH 0.1
#define V_THRESH 0.2

//分布参数??????
#define LAMBDA 20

/***********************Structures************************/

//直方图用NH * NS + NV的形式表示，大于阈值的像素值用NH * NS表示，“无色”的像素值用NV表示
typedef struct histogram{
	float histo[NH * NS + NV];    //直方图数组
	int n; //数组点的长度
}histogram;

/**********************Function Definitions******************/

/*
	将BGR图片转换成HSV颜色模型

	@param img:待转换的图片

	@return :转换为一个3通道，32bit的HSV图像，
			 S and V values in the range [0,1] and H value in the range [0,360]
*/

IplImage* bgr2hsv(IplImage* img);

/*
	计算HSV直方图的bin值

	@param h Hue
	@param s Saturation
	@param v Value

	@return 返回与HSV对应的bin的索引值
*/
int histo_bin(float h, float s, float v);

/*
	对于给定的图像数组，计算其累积的直方图

	@param imgs an array of images over which to compute a cumulative histogram;
	each must have been converted to HSV colorspace using bgr2hsv()
	@param n the number of images in imgs

	@return 返回imgs的HSV直方图（无归一化）
*/
histogram* calc_histogram(IplImage** imgs,int n);

/**
	Normalizes a histogram so all bins sum to 1.0

	@param histo a histogram
*/
void normalize_histogram(histogram* histo);

/**
	依据巴氏距离计算俩个直方图的相似度

	@param h1 first histogram; should be normalized
    @param h2 second histogram; should be normalized
   
    @return Rerns a squared distance9（距离的平方） based on the Battacharyya similarity
     coefficient between  h1 and  h2
*/
float histo_dist_sq(histogram* h1,histogram* h2);

/**
	计算在与给定直方图相似的图像

	@param img image that has been converted to HSV colorspace using bgr2hsv()
	@param r row location of center of window around which to compute likelihood
	@param c col location of center of window around which to compute likelihood
	@param w width of region over which to compute likelihood
	@param h height of region over which to compute likelihood
	@param ref_histo reference histogram for a player; must have been
	normalized with normalize_histogram()

	@return 返回相似值
*/
float likelihood(IplImage* img,int r,int c,int w,int h,histogram* ref_histo);

/**
	计算感兴趣区域中每一个像素的相似度

	@param img the image for which likelihood is to be computed; should have
	been converted to HSV colorspace using bgr2hsv()
	@param w width of region over which to compute likelihood
	@param h height of region over which to compute likelihood
	@param ref_histo reference histogram for a player; must have been
	normalized with normalize_histogram()

	@return Returns a single-channel, 32-bit floating point image containing
	the likelihood of every pixel location in \a img normalized so that the
	sum of likelihoods is 1.
*/
IplImage* likelihood_image(IplImage* img, int w, int h, histogram* ref_histo);

/**
	将直方图的数据导出到文件

	0 <h_0>
	...
	<i> <h_i>
	...
	<n> <h_n>

	Where n is the number of histogram bins and h_i, i = 1..n are
	floating point bin values

	@param histo histogram to be exported
	@param filename name of file to which histogram is to be exported

	@return Returns 1 on success or 0 on failure
*/
int export_histogram(histogram* histo, char* filename);



#endif


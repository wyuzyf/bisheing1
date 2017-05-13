/*

Tracker based on Kernelized Correlation Filter (KCF) [1] and Circulant Structure with Kernels (CSK) [2].
CSK is implemented by using raw gray level features, since it is a single-channel filter.
KCF is implemented by using HOG features (the default), since it extends CSK to multiple channels.

[1] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.

[2] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.

Authors: Joao Faro, Christian Bailer, Joao F. Henriques
Contacts: joaopfaro@gmail.com, Christian.Bailer@dfki.de, henriques@isr.uc.pt
Institute of Systems and Robotics - University of Coimbra / Department Augmented Vision DFKI


Constructor parameters, all boolean:
hog: use HOG features (default), otherwise use raw pixels
fixed_window: fix window size (default), otherwise use ROI size (slower but more accurate)
multiscale: use multi-scale tracking (default; cannot be used with fixed_window = true)

Default values are set for all properties of the tracker depending on the above choices.
Their values can be customized further before calling init():
interp_factor: linear interpolation factor for adaptation
sigma: gaussian kernel bandwidth
lambda: regularization
cell_size: HOG cell size
padding: horizontal area surrounding the target, relative to its size
output_sigma_factor: bandwidth of gaussian target
template_size: template size in pixels, 0 to use ROI size
scale_step: scale step for multi-scale estimation, 1 to disable it
scale_weight: to downweight detection scores of other scales for added stability

For speed, the value (template_size/cell_size) should be a power of 2 or a product of small prime numbers.

Inputs to init():
image is the initial frame.
roi is a cv::Rect with the target positions in the initial frame

Inputs to update():
image is the current frame.

Outputs of update():
cv::Rect with target positions for the current frame


By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install,
copy or use the software.


License Agreement
For Open Source Computer Vision Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the names of the copyright holders nor the names of the contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall copyright holders or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#pragma once


#include "tracker.h"

#ifndef _OPENCV_KCFTRACKER_HPP_
#define _OPENCV_KCFTRACKER_HPP_
#endif

//继承Tracker
class KCFTracker : public Tracker
{
public:
	// Constructor  构造KCF跟踪器的类
	KCFTracker(bool hog = true,   //使用HOG特征
		bool fixed_window = true,   //使用固定窗口
		bool multiscale = true,    //使用多尺度
		bool lab = true);    //使用Lab颜色空间特征

	
	// Initialize tracker 
	//初始化跟踪器，roi是目标初始框的引用，image是进入跟踪的第一幅图像
	virtual void init(const cv::Rect &roi, cv::Mat image);

	
	// Update position based on the new frame
	//基于新一帧的图像更新目标位置，image是新一帧图像
	//多尺度定义在这里，通过检测一个大点和一个小点的尺度，比较三个峰值的结果来进行多尺度的适应
	virtual cv::Rect update(cv::Mat image);  // cv::Rect是返回类型


	float interp_factor; // linear interpolation factor for adaptation
						//自适应的线性插值因子，会因为HOG,Lab的选择变化
	float sigma; // gaussian kernel bandwidth
				//高斯卷积核带宽，会因为HOG,Lab的选择变化
	float lambda; // regularization
					//正则化参数，0.0001
	int cell_size; // HOG cell size 
					//细胞单元尺寸
	int cell_sizeQ; // cell size^2, to avoid repeated operations
					//细胞单元内像素，比如6*6的像素数目
	float padding; // extra area surrounding the target
				//目标扩展出来的区域
	float output_sigma_factor; // bandwidth of gaussian target
							//高斯目标的带宽
	int template_size; // template size
	                //模板大小，在计算_tmpl_sz时，  
                    // 较大变成被归一成96，而较小边长按比例缩小 
	float scale_step; // scale step for multi-scale estimation
					//多尺度估计的尺度步长
	float scale_weight;  // to downweight detection scores of other scales for added stability
						//为了增加其它尺度检测时的稳定性，给检测结果做一定的衰减比，为原来的0.95
protected:
	// Detect object in the current frame.
	//检测当前帧的目标
	//z是前一帧的训练或者是第一帧的初始化结果，x是当前帧尺度下的特征，peak_value是检测结果峰值
	cv::Point2f detect(cv::Mat z, cv::Mat x, float &peak_value);

	
	// train tracker with a single image
	//使用当前图像的检测结果进行训练，x是当前帧尺度下的特征，train_interp_factor是interp_factor
	void train(cv::Mat x, float train_interp_factor);

	// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, 
	//which must both be MxN. They must also be periodic (ie., pre-processed with a cosine window).
	//使用带宽SIGMA计算高斯卷积核用于所有图像x和y之间的相对位移，
	//必须是M*N大小，二者必须是周期性的（通过一个cos(余弦)窗口进行预处理）
	cv::Mat gaussianCorrelation(cv::Mat x1, cv::Mat x2);

	
	// Create Gaussian Peak. Function called only in the first frame.
	//创建高斯峰值函数，此函数只在第一帧时调用
	cv::Mat createGaussianPeak(int sizey, int sizex);

	// Obtain sub-window from image, with replication-padding and extract features
	//从图像得到2.5倍子窗口（特征），通过赋值填充并验证
	cv::Mat getFeatures(const cv::Mat & image, bool inithann, float scale_adjust = 1.0f);

	// Initialize Hanning window. Function called only in the first frame.
	//初始化Hanning窗口？？？
	void createHanningMats();

	// Calculate sub-pixel peak for one dimension
	//计算一维亚像素峰值，俩个物理像素之间还有sub-pixel
	float subPixelPeak(float left, float center, float right);

	cv::Mat _alphaf;  //初始化/训练结果alpha,用于检测部分中结果的计算，就是论文中证明了半天的的alpha
	cv::Mat _prob;   //初始化结果概率，不用更改，用于训练，即预测到的目标位置
	cv::Mat _tmpl;  //初始化/训练的结果，用于detect的z，即论文中的输入图像z
	
	cv::Mat _num;   //
	cv::Mat _den;
	
	cv::Mat _labCentroids; //Lab质心数组

private:
	int size_patch[3];   //HOG特征的sizeY，sizeX，numFeatures（维数）
	cv::Mat hann;    //createHanningMats()的计算结果
	cv::Size _tmpl_sz;  //HOG细胞元对应的数组大小
	float _scale;  //修正成_tmpl_sz后的尺度大小，尺度不是指物体的尺寸大小，是某一特征的参数
	
	int _gaussian_size; //计算得到的高斯大小
	
	bool _hogfeatures;   //HOG标志位
	bool _labfeatures;  //lab标志位
};


#include <iostream>
#ifndef _KCFTRACKER_HEADERS
#include "kcftracker.h"
#include "ffttools.h"
#include "recttools.h"
#include "fhog.h"
#include "labdata.h"

#endif


// Constructor 构造函数，变量初始赋值
KCFTracker::KCFTracker(bool hog, bool fixed_window, bool multiscale, bool lab)
{
	// Parameters equal in all cases
	lambda = 0.0001;
	padding = 2.5;
	output_sigma_factor = 0.125;  //回归目标相对于目标大小的空间带宽？？？

	if (hog) {    // HOG
		// VOT
		//interp_factor = 0.012;
		//sigma = 0.6; 
		// TPAMI
		interp_factor = 0.02;
		sigma = 0.5;
		cell_size = 8;
		_hogfeatures = true;

		if (lab) {  // lab
			interp_factor = 0.005;
			sigma = 0.4;
			//output_sigma_factor = 0.025;
			output_sigma_factor = 0.1; 

			_labfeatures = true;
			_labCentroids = cv::Mat(nClusters, 3, CV_32FC1, &data);
			cell_sizeQ = cell_size*cell_size;
		}
		else{
			_labfeatures = false;
		}
	} // (hog)
	else {   // RAW
		interp_factor = 0.075;
		sigma = 0.2;
		cell_size = 1;
		_hogfeatures = false;

		if (lab) {
			printf("Lab features are only used with HOG features.\n");
			_labfeatures = false;
		}
	}

	if (multiscale) { // multiscale
		template_size = 96;
		scale_step = 1.05;
		scale_weight = 0.95;
		if (!fixed_window) {
			// 多尺度 只支持 固定窗口
			//printf("Multiscale does not support non-fixed window.\n");
			fixed_window = true;
		}
	}
	else if (fixed_window) {  // fit correction without multiscale
		template_size = 96;
		//template_size = 100;
		scale_step = 1;
	}
	else {
		template_size = 1;
		scale_step = 1;
	}
}

// Initialize tracker 
void KCFTracker::init(const cv::Rect &roi, cv::Mat image)
{
	_roi = roi;
	assert(roi.width >= 0 && roi.height >= 0);   // assert (condition);  condition is false, the program will abort/stop
	_tmpl = getFeatures(image, 1); // template: 31 rows * (sizeX * sizeY) cols
	// alpha里面的Y(即_prob)到底是什么？按照原来的公式应该是标签的，但最终怎么成了一个高斯的分布？
	//（好像有点懂了，因为他不是像别人那样直接给目标标签定为1或者是0，而是一个确信度，难道这就是回归的思想）
	_prob = createGaussianPeak(size_patch[0], size_patch[1]);
	_alphaf = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));

	train(_tmpl, 1.0); // train with initial frame
}

// Create Gaussian Peak. Function called only in the first frame.
// 高斯金字塔，这个函数的作用搞不懂，样本对目标的相似度
// 首先将范围变成以target中心为原点的分布，然后表示出目标中心可能出现的概率分布，变成了区间【0，1】；接下来变换到频域里面去。
// 由此可以看出公式中的y不是我们平时使用的 1 或者是 -1，而是一个可能出现的概率标签。即经过傅里叶变换后的图像
//现象是突出目标，模糊背景
cv::Mat KCFTracker::createGaussianPeak(int sizey, int sizex)
{
	cv::Mat_<float> res(sizey, sizex); // height = sizey, width = sizex
	// target中心位置
	int syh = (sizey) / 2;
	int sxh = (sizex) / 2;

	// http://www.tuicool.com/articles/eArINbY
	float output_sigma = std::sqrt((float)sizex * sizey) / padding * output_sigma_factor;
	// output_sigma 决定了高斯函数的宽度
	// output_sigma 越大，高斯滤波器的频带就越宽，平滑程度就越好
	float mult = -0.5 / (output_sigma * output_sigma);

	for (int i = 0; i < sizey; i++)
	for (int j = 0; j < sizex; j++)
	{
		int ih = i - syh;
		int jh = j - sxh;
		res(i, j) = std::exp(mult * (float)(ih * ih + jh * jh));
	}
	return FFTTools::fftd(res);
}




/*-----------------------------------我是分割线----------------------------------------------------*/
//分割线之间的是模型更新,训练模块


// Update position based on the new frame
//基于当前帧更新目标位置
cv::Rect KCFTracker::update(cv::Mat image)
{
	//修正边界，限制框选区域的位置
	if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 1;
	if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 1;
	if (_roi.x >= image.cols - 1) _roi.x = image.cols - 2;
	if (_roi.y >= image.rows - 1) _roi.y = image.rows - 2;

	//（1）跟踪框中心
	float cx = _roi.x + _roi.width / 2.0f;
	float cy = _roi.y + _roi.height / 2.0f;

	//尺度不变时检测峰值结果
	float peak_value;
	cv::Point2f res = detect(_tmpl, getFeatures(image, 0, 1.0f), peak_value);

	if (scale_step != 1) {
		// Test at a smaller _scale
		float new_peak_value;
		cv::Point2f new_res = detect(_tmpl, getFeatures(image, 0, 1.0f / scale_step), new_peak_value);

		if (scale_weight * new_peak_value > peak_value) {
			res = new_res;
			peak_value = new_peak_value;
			_scale /= scale_step;
			_roi.width /= scale_step;
			_roi.height /= scale_step;
		}

		// Test at a bigger _scale
		new_res = detect(_tmpl, getFeatures(image, 0, scale_step), new_peak_value);

		if (scale_weight * new_peak_value > peak_value) {
			res = new_res;
			peak_value = new_peak_value;
			_scale *= scale_step;
			_roi.width *= scale_step;
			_roi.height *= scale_step;
		}
	}

	// Adjust by cell size and _scale
	_roi.x = cx - _roi.width / 2.0f + ((float)res.x * cell_size * _scale);
	_roi.y = cy - _roi.height / 2.0f + ((float)res.y * cell_size * _scale);

	if (_roi.x >= image.cols - 1) _roi.x = image.cols - 1;
	if (_roi.y >= image.rows - 1) _roi.y = image.rows - 1;
	if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 2;
	if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 2;

	assert(_roi.width >= 0 && _roi.height >= 0);
	cv::Mat x = getFeatures(image, 0);
	train(x, interp_factor);

	return _roi;
}


//目标模型是由学习“目标外观” 和 变换的“分类器参数α”组成
//这里对应的是 train_interp_factor 和 alpha，train_interp_factor是由自己定的，这里设定为1.0
// train tracker with a single image
void KCFTracker::train(cv::Mat x, float train_interp_factor)
{
	using namespace FFTTools;

	cv::Mat k = gaussianCorrelation(x, x);
	cv::Mat alphaf = complexDivision(_prob, (fftd(k) + lambda));

	_tmpl = (1 - train_interp_factor) * _tmpl + (train_interp_factor)* x;
	_alphaf = (1 - train_interp_factor) * _alphaf + (train_interp_factor)* alphaf;

}


/*-----------------------------------我是分割线----------------------------------------------------*/

//分割线之间的是和跟踪模块有关的函数

// Obtain sub-window from image, with replication-padding and extract features
cv::Mat KCFTracker::getFeatures(const cv::Mat &image, bool inithann, float scale_adjust)
{
	// 需要提取出来的区域
	cv::Rect extracted_roi;

	// _roi 的中心位置
	float cx = _roi.x + _roi.width / 2;
	float cy = _roi.y + _roi.height / 2;

	if (inithann)
	{
		// 扩大区域
		int padded_w = _roi.width * padding;
		int padded_h = _roi.height * padding;

		if (template_size > 1)
		{	// Fit largest dimension to the given template size 
			// 将 padded_w 或 padded_h 中较大值调整为等于template_size，较小值也同比例变化
			// 变化尺度因子： _scale
			if (padded_w >= padded_h)  //fit to width
				_scale = padded_w / (float)template_size;
			else    // fit to height
				_scale = padded_h / (float)template_size;

			_tmpl_sz.width = padded_w / _scale;
			_tmpl_sz.height = padded_h / _scale;
		}
		else
		{  //No template size given, use ROI size
			_tmpl_sz.width = padded_w;
			_tmpl_sz.height = padded_h;
			_scale = 1;
			// original code from paper:
			/*
			if (sqrt(padded_w * padded_h) >= 100) {   //Normal size
			_tmpl_sz.width = padded_w;
			_tmpl_sz.height = padded_h;
			_scale = 1;
			}
			else {   //ROI is too big, track at half size
			_tmpl_sz.width = padded_w / 2;
			_tmpl_sz.height = padded_h / 2;
			_scale = 2;
			}*/
		}

		if (_hogfeatures)
		{
			// Round to cell size and also make it even
			// _tmpl_sz 的宽度和高度的大小应该是(2*cell size)的整数倍，cell size的偶数倍
			// http://blog.csdn.net/ttransposition/article/details/41806601#t0
			_tmpl_sz.width = (((int)(_tmpl_sz.width / (2 * cell_size))) * 2 * cell_size) + cell_size * 2;
			_tmpl_sz.height = (((int)(_tmpl_sz.height / (2 * cell_size))) * 2 * cell_size) + cell_size * 2;
		}
		else
		{	//Make number of pixels even (helps with some logic involving half-dimensions)
			_tmpl_sz.width = (_tmpl_sz.width / 2) * 2;
			_tmpl_sz.height = (_tmpl_sz.height / 2) * 2;
		}
	}

	extracted_roi.width = scale_adjust * _scale * _tmpl_sz.width;
	extracted_roi.height = scale_adjust * _scale * _tmpl_sz.height;

	//center roi with new size 
	extracted_roi.x = cx - extracted_roi.width / 2;
	extracted_roi.y = cy - extracted_roi.height / 2;

	cv::Mat FeaturesMap;
	// 从image截取出区域为extracted_roi的图片
	cv::Mat z = RectTools::subwindow(image, extracted_roi, cv::BORDER_REPLICATE);

	if (z.cols != _tmpl_sz.width || z.rows != _tmpl_sz.height)
		cv::resize(z, z, _tmpl_sz);  // resize 所提取出的区域

	// HOG features
	if (_hogfeatures)
	{
		IplImage z_ipl = z; // Mat 转 IplImage
		CvLSVMFeatureMapCaskade *map;

		// 统计梯度方向直方图 , numFeatures = NUM_SECTOR * 3 
		getFeatureMaps(&z_ipl, cell_size, &map);

		// 归一化与截断, numFeatures = NUM_SECTOR * 3 * 4
		normalizeAndTruncate(map, 0.2);

		// PCA降维,  numFeatures = NUM_SECTOR * 3 + 4
		PCAFeatureMaps(map);

		size_patch[0] = map->sizeY;
		size_patch[1] = map->sizeX;
		size_patch[2] = map->numFeatures;

		// Procedure do deal with cv::Mat multichannel bug
		FeaturesMap = cv::Mat(cv::Size(map->numFeatures, map->sizeX * map->sizeY), CV_32F, map->map);  // 行号表示cell的索引
		FeaturesMap = FeaturesMap.t(); // 矩阵转置 第i行表示第i个特征，第j列表示第j个单元格(cell)
		freeFeatureMapObject(&map);

		// Lab features
		if (_labfeatures)
		{
			cv::Mat imgLab;
			cvtColor(z, imgLab, CV_BGR2Lab);
			unsigned char *input = (unsigned char*)(imgLab.data);

			//Sparse output vector 稀疏输出向量
			cv::Mat outputLab = cv::Mat(_labCentroids.rows, size_patch[0] * size_patch[1], CV_32F, float(0));

			int cntCell = 0;
			
			// Iterate through each cell通过每个细胞迭代
			for (int cY = cell_size; cY < z.rows - cell_size; cY += cell_size){
				for (int cX = cell_size; cX < z.cols - cell_size; cX += cell_size){
					// Iterate through each pixel of cell (cX,cY)通过每个像素迭代
					for (int y = cY; y < cY + cell_size; ++y){
						for (int x = cX; x < cX + cell_size; ++x){
							// Lab components for each pixel每个像素的lab特征
							float l = (float)input[(z.cols * y + x) * 3];
							float a = (float)input[(z.cols * y + x) * 3 + 1];
							float b = (float)input[(z.cols * y + x) * 3 + 2];

							// Iterate trough each centroid通过每个质心迭代
							float minDist = FLT_MAX;
							int minIdx = 0;
							float *inputCentroid = (float*)(_labCentroids.data);
							for (int k = 0; k < _labCentroids.rows; ++k)
							{
								float dist = ((l - inputCentroid[3 * k]) * (l - inputCentroid[3 * k]))
									+ ((a - inputCentroid[3 * k + 1]) * (a - inputCentroid[3 * k + 1]))
									+ ((b - inputCentroid[3 * k + 2]) * (b - inputCentroid[3 * k + 2]));
								if (dist < minDist)
								{
									minDist = dist;
									minIdx = k;
								}
							}
							// Store result at output
							outputLab.at<float>(minIdx, cntCell) += 1.0 / cell_sizeQ;
							//((float*) outputLab.data)[minIdx * (size_patch[0]*size_patch[1]) + cntCell] += 1.0 / cell_sizeQ; 
						}
					}
					cntCell++;
				}
			}
			// Update size_patch[2] and add features to FeaturesMap
			size_patch[2] += _labCentroids.rows;
			FeaturesMap.push_back(outputLab);
		}//lab feature
	}//hog feature
	else {
		FeaturesMap = RectTools::getGrayImage(z);
		FeaturesMap -= (float) 0.5; // In Paper;
		size_patch[0] = z.rows;
		size_patch[1] = z.cols;
		size_patch[2] = 1;
	}

	if (inithann)
		createHanningMats();
	FeaturesMap = hann.mul(FeaturesMap);  //执行两个矩阵按元素相乘
	return FeaturesMap;
}


//第二大步
//Initialize Hanning window（余弦cos窗口）. Function called only in the first frame.
//余弦窗的作用是对循环移位后的图像样本进行平滑加权
//因为循环偏移采样得到的样本在靠近边界处会有一些明显的边界线，导致图像变得不平滑
//因此需要乘以一个余弦窗来降低边缘像素的权重
//这里用的是opencv中自带的
void KCFTracker::createHanningMats()
{
	cv::Mat hann1t = cv::Mat(cv::Size(size_patch[1], 1), CV_32F, cv::Scalar(0));	//size_patch[1] = map->sizeX;   行向量
	cv::Mat hann2t = cv::Mat(cv::Size(1, size_patch[0]), CV_32F, cv::Scalar(0)); //size_patch[0] = map->sizeY;	列向量

	// 一个维度上的表示是这样的： w(n)=0.5（1−cos(2*pi*n/N) ) ,0≤n≤N
	for (int i = 0; i < hann1t.cols; i++)
		hann1t.at<float >(0, i) = 0.5 * (1 - std::cos(2 * CV_PI * i / (hann1t.cols - 1)));  //CV_PI = 3.14159265358979323846
	for (int i = 0; i < hann2t.rows; i++)
		hann2t.at<float >(i, 0) = 0.5 * (1 - std::cos(2 * CV_PI * i / (hann2t.rows - 1)));
	// 为了消除边缘的效应和强调中心，使用一个窗口。
	cv::Mat hann2d = hann2t * hann1t;
	// HOG features
	if (_hogfeatures)
	{
		// Procedure do deal with cv::Mat multichannel bug
		cv::Mat hann1d = hann2d.reshape(1, 1); // 第一个参数channels = 1 , 第二个参数rows = 1, OPENCV是行优先

		hann = cv::Mat(cv::Size(size_patch[0] * size_patch[1], size_patch[2]), CV_32F, cv::Scalar(0));
		for (int i = 0; i < size_patch[2]; i++)
		for (int j = 0; j<size_patch[0] * size_patch[1]; j++)
			hann.at<float>(i, j) = hann1d.at<float>(0, j);
	}
	// Gray features
	else
		hann = hann2d;
}

//第三大步
//Detect object in the current frame.
//z为前一帧样本，x为当前帧图像，peak_value为输出的峰值
cv::Point2f KCFTracker::detect(cv::Mat z, cv::Mat x, float &peak_value)
{
	using namespace FFTTools;

	cv::Mat k = gaussianCorrelation(x, z); //计算x（当前）与z（样本）的核相关矩阵
	cv::Mat res = (real(fftd(complexMultiplication(_alphaf, fftd(k)), true)));//计算响应值

	//minMaxLoc only accepts doubles for the peak, and integer points for the coordinates 
	//minMaxLoc只接受峰值的两倍，坐标的整数点
	double pv;		// 矩阵中元素最大值
	cv::Point2i pi; // 矩阵中元素最大值在矩阵中的位置 
	cv::minMaxLoc(res, NULL, &pv, NULL, &pi);//计算响应的极大值和极小值，并得到其所在位置

	peak_value = (float)pv;

	//subpixel peak estimation, coordinates will be non-integer
	//子像素峰值估计，坐标将为非整数
	cv::Point2f  p((float)pi.x, (float)pi.y);

	if (pi.x > 0 && pi.x < res.cols - 1)
		p.x += subPixelPeak(res.at<float>(pi.y, pi.x - 1), peak_value, res.at<float>(pi.y, pi.x + 1));

	if (pi.y > 0 && pi.y < res.rows - 1)
		p.y += subPixelPeak(res.at<float>(pi.y - 1, pi.x), peak_value, res.at<float>(pi.y + 1, pi.x));

	p.x -= (res.cols) / 2;
	p.y -= (res.rows) / 2;

	return p;
}

// Calculate sub-pixel peak for one dimension
float KCFTracker::subPixelPeak(float left, float center, float right)
{
	float divisor = 2 * center - right - left;

	if (divisor == 0)
		return 0;

	return 0.5 * (right - left) / divisor;
}

/*-----------------------------------我是分割线----------------------------------------------------*/

// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN. 两矩阵大小相同
// They must also be periodic (ie., pre-processed with a cosine window).
//计算相关性
cv::Mat KCFTracker::gaussianCorrelation(cv::Mat x1, cv::Mat x2)
{
	using namespace FFTTools;
	cv::Mat c = cv::Mat(cv::Size(size_patch[1], size_patch[0]), CV_32F, cv::Scalar(0));
	cv::Mat caux;
	cv::Mat x1aux;
	cv::Mat x2aux;

	// HOG features
	if (_hogfeatures)
	{
		for (int i = 0; i < size_patch[2]; i++) // size_patch[2] = 31 , map->numFeatures
		{
			x1aux = x1.row(i);   // Procedure do deal with cv::Mat multichannel bug
			x1aux = x1aux.reshape(1, size_patch[0]); // channels = 1, rows = size_patch[0], cols = x1aux.cols / rows
			x2aux = x2.row(i).reshape(1, size_patch[0]); // x2.row(i)是行向量，reshape之后变成 map->sizeY * map->sizeX 的矩阵
			cv::mulSpectrums(fftd(x1aux), fftd(x2aux), caux, 0, true); // 两张频谱图中每一个元素相乘， 相乘之前取fftd(x2aux)共轭 fftd(x1aux).*conj( fftd(x2aux) ) 
			caux = fftd(caux, true); // 傅里叶逆变换
			rearrange(caux);  // 对傅立叶变换的图像进行重排 
			caux.convertTo(caux, CV_32F);
			c = c + real(caux);
		}
	}
	// Gray features
	else
	{
		cv::mulSpectrums(fftd(x1), fftd(x2), c, 0, true);
		c = fftd(c, true);
		rearrange(c);
		c = real(c);
	}
	cv::Mat d;
	// cv::Mat matT = ((cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0]) - 2. * c) / (size_patch[0] * size_patch[1] * size_patch[2]);
	// 取src1 和 src2 对应的元素的最大值组成输出同样大小的矩阵 d 
	// cv::sum(x1.mul(x1))[0] 矩阵范数，所有cv::sum()表示元素之和
	cv::max(((cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0]) - 2. * c) / (size_patch[0] * size_patch[1] * size_patch[2]), 0, d); //?????

	cv::Mat k;
	cv::exp((-d / (sigma * sigma)), k);
	return k;
}


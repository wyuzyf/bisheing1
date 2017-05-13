
//快速傅里叶工具,自己实现的傅里叶变换

#pragma once    //预编译一次

#include <cv.h>

#ifndef _OPENCV_FFFTOOLS_HPP_H
#define _OPENCV_FFFTOOLS_HPP_H
#endif

//(2)http://johnhany.net/2013/11/dft-based-text-rotation-correction/#imageclose-380
//(1)http://blog.csdn.net/zhoufan900428/article/details/45845957/
//(3)http://blog.csdn.net/chenjiazhou12/article/details/21240647
//(4)官方 http://www.opencv.org.cn/opencvdoc/2.3.2/html/doc/tutorials/core/discrete_fourier_transform/discrete_fourier_transform.html
namespace FFTTools
{
	//以前的声明，避免出现警告
	cv::Mat fftd(cv::Mat img, bool backwards = false);
	cv::Mat real(cv::Mat img);
	cv::Mat imag(cv::Mat img);
	cv::Mat magnitude(cv::Mat img);
	cv::Mat complexMultiplication(cv::Mat a, cv::Mat b);
	cv::Mat complexDivision(cv::Mat a, cv::Mat b);
	//void rearrange(cv::Mat &img);
	void normalizedLogTransform(cv::Mat &img);


	cv::Mat fftd(cv::Mat img, bool backwards)
	{
		if (img.channels() == 1)
		{
			cv::Mat planes[] = { cv::Mat_<float>(img), cv::Mat_<float>::zeros(img.size()) };
			//cv::Mat planes[] = {cv::Mat_<double> (img), cv::Mat_<double>::zeros(img.size())};
			cv::merge(planes, 2, img);  //合并多个单通道到一个多通道 img: Complex plane to contain the DFT coefficients {[0]-Real,[1]-Img}
		}
		cv::dft(img, img, backwards ? (CV_DXT_INVERSE | CV_DXT_SCALE) : CV_DXT_FORWARD);  //进行傅里叶变换

		return img;
	}

	cv::Mat real(cv::Mat img)  //实部
	{
		std::vector<cv::Mat> planes;
		cv::split(img, planes);
		return planes[0];
	}

	cv::Mat imag(cv::Mat img)   //虚部
	{
		std::vector<cv::Mat> planes;
		cv::split(img, planes);
		return planes[1];
	}

	cv::Mat magnitude(cv::Mat img)  //计算幅度,用幅度图表示傅里叶的变换结果（傅里叶谱）
	{
		cv::Mat res;
		std::vector<cv::Mat> planes;
		cv::split(img, planes); // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
		if (planes.size() == 1) res = cv::abs(img);
		else if (planes.size() == 2) cv::magnitude(planes[0], planes[1], res); // planes[0] = magnitude
		else CV_Assert(0);
		return res;
	}

	// 两矩阵对应元素相乘
	cv::Mat complexMultiplication(cv::Mat a, cv::Mat b)
	{
		std::vector<cv::Mat> pa;
		std::vector<cv::Mat> pb;
		cv::split(a, pa);
		cv::split(b, pb);

		std::vector<cv::Mat> pres;
		pres.push_back(pa[0].mul(pb[0]) - pa[1].mul(pb[1]));
		pres.push_back(pa[0].mul(pb[1]) + pa[1].mul(pb[0]));

		cv::Mat res;
		cv::merge(pres, res);

		return res;
	}

	// 两矩阵对应元素相除
	cv::Mat complexDivision(cv::Mat a, cv::Mat b)
	{
		std::vector<cv::Mat> pa;
		std::vector<cv::Mat> pb;
		// 通道分离
		cv::split(a, pa);
		cv::split(b, pb);

		/* ===============================================================

		a0 + a1*i    (a0 + a1*i)(b0 - b1*i)    a0*b0 + a1*b1 + (a1*b0 - a0*b1)*i
		---------  = ---------------------- = -----------------------------------
		b0 + b1*i    (b0 + b1*i)(b0 - b1*i)             b0*b0 + b1*b1

		=============================================================== */

		cv::Mat divisor = 1. / (pb[0].mul(pb[0]) + pb[1].mul(pb[1]));

		std::vector<cv::Mat> pres;

		pres.push_back((pa[0].mul(pb[0]) + pa[1].mul(pb[1])).mul(divisor));
		pres.push_back((pa[1].mul(pb[0]) - pa[0].mul(pb[1])).mul(divisor));

		cv::Mat res;
		// 合并通道
		cv::merge(pres, res);
		return res;
	}

	//重新排列傅里叶图像的象限，使原点在图像中心
	//dft()直接获得的结果中，低频部分位于四角，高频部分位于中间。
	//习惯上会把图像做四等份，互相对调，使低频部分位于图像中心，也就是让频域原点位于中心。
	void rearrange(cv::Mat &img)
	{
		// img = img(cv::Rect(0, 0, img.cols & -2, img.rows & -2));
		int cx = img.cols / 2;
		int cy = img.rows / 2;

		cv::Mat q0(img, cv::Rect(0, 0, cx, cy)); // Top-Left - Create a ROI per quadrant
		cv::Mat q1(img, cv::Rect(cx, 0, cx, cy)); // Top-Right
		cv::Mat q2(img, cv::Rect(0, cy, cx, cy)); // Bottom-Left
		cv::Mat q3(img, cv::Rect(cx, cy, cx, cy)); // Bottom-Right

		cv::Mat tmp; // swap quadrants (Top-Left with Bottom-Right)
		q0.copyTo(tmp);
		q3.copyTo(q0);
		tmp.copyTo(q3);
		q1.copyTo(tmp); // swap quadrant (Top-Right with Bottom-Left)
		q2.copyTo(q1);
		tmp.copyTo(q2);
	}
	/*
	template < typename type>
	cv::Mat fouriertransFull(const cv::Mat & in)  //完整傅里叶变换的过程
	{
	return fftd(in);

	cv::Mat planes[] = {cv::Mat_<type > (in), cv::Mat_<type>::zeros(in.size())};
	cv::Mat t;
	assert(planes[0].depth() == planes[1].depth());
	assert(planes[0].size == planes[1].size);
	cv::merge(planes, 2, t);
	cv::dft(t, t);

	//cv::normalize(a, a, 0, 1, CV_MINMAX);
	//cv::normalize(t, t, 0, 1, CV_MINMAX);

	// cv::imshow("a",real(a));
	//  cv::imshow("b",real(t));
	// cv::waitKey(0);

	return t;
	}*/

	//对数组的每一个元素去自然对数
	void normalizedLogTransform(cv::Mat &img)
	{
		img = cv::abs(img);
		img += cv::Scalar::all(1);
		cv::log(img, img);
		// cv::normalize(img, img, 0, 1, CV_MINMAX); //矩阵归一化
	}

}



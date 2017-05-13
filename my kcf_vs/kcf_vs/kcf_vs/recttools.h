
#include <cv.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#ifndef _OPENCV_RECTTOOLS_HPP_
#define _OPENCV_RECTTOOLS_HPP_
#endif

namespace RectTools
{
	template <typename t> inline cv::Vec<t, 2> center(const cv::Rect_<t> &rect)
	{
		return cv::Vec<t, 2> (rect.x + rect.width / (t) 2, rect.y + rect.height / (t) 2);  //中心点坐标
	}

	template <typename t> inline t x2(const cv::Rect_<t> &rect)
	{
		return rect.x + rect.width;  //x坐标
	}

	template <typename t> inline t y2(const cv::Rect_<t> &rect)
	{
		return rect.y + rect.height;  //y坐标
	}

	//调整大小
	template <typename t> inline void resize(cv::Rect_<t> &rect, float scalex, float scaley)
	{
		if (!scaley) 
			scaley = scalex;
		rect.x -= rect.width * (scalex - 1.f) / 2.f;
		rect.width *= scalex;

		rect.y -= rect.height * (scaley - 1.f) / 2.f;
		rect.height *= scaley;
	}

	//限制
	template <typename t> inline void limit(cv::Rect_<t> &rect, cv::Rect_<t> limit)
	{
		// 矩形框rect右边界的x限制为(limit.x+limit.width)
		if (rect.x + rect.width > limit.x + limit.width)
			rect.width = (limit.x + limit.width - rect.x); // rect.width = rect.width - (rect.x + rect.width -(limit.x + limit.width))

		// 矩形框rect下边界的y限制为(limit.y+limit.height)
		if (rect.y + rect.height > limit.y + limit.height)
			rect.height = (limit.y + limit.height - rect.y);

		// 限制矩形框rect的左边界
		if (rect.x < limit.x)
		{
			rect.width -= (limit.x - rect.x);
			rect.x = limit.x;
		}

		// 限制矩形框rect的上边界
		if (rect.y < limit.y)
		{
			rect.height -= (limit.y - rect.y);
			rect.y = limit.y;
		}

		if (rect.width<0)
			rect.width = 0;

		if (rect.height<0)
			rect.height = 0;
	}

	template <typename t> inline void limit(cv::Rect_<t> &rect, t width, t height, t x = 0, t y = 0)
	{
		limit(rect, cv::Rect_<t>(x, y, width, height));
	}

	//实际的边界坐标？？？？？
	template <typename t> inline cv::Rect getBorder(const cv::Rect_<t> &original, cv::Rect_<t> &limited)
	{
		cv::Rect_<t> res;
		res.x = limited.x - original.x;
		res.y = limited.y - original.y;
		//res.width = x2(original) - x2(limited);  //rect.x + rect.width
		res.width = (original.x + original.width) - (limited.x + limited.width);
		//res.height = y2(original) - y2(limited);
		res.height = (original.y + original.height) - (limited.y + limited.height);
		CV_Assert(res.x >= 0 && res.y >= 0 && res.width >= 0 && res.height >= 0);
		return res;
	}

	inline cv::Mat subwindow(const cv::Mat &in, const cv::Rect &window, int borderType = cv::BORDER_CONSTANT)
	{
		cv::Rect cutWindow = window;
		RectTools::limit(cutWindow, in.cols, in.rows);
		if (cutWindow.height <= 0 || cutWindow.width <= 0)
			CV_Assert(0); //return cv::Mat(window.height,window.width,in.type(),0) ;
		cv::Rect border = RectTools::getBorder(window, cutWindow);
		cv::Mat res = in(cutWindow);

		if (border != cv::Rect(0, 0, 0, 0))
			cv::copyMakeBorder(res, res, border.y, border.height, border.x, border.width, borderType);
		return res;
	}

	//得到灰色图
	inline cv::Mat getGrayImage(cv::Mat img)
	{
		//cv::cvtColor(img, img, COLOR_BGR2GRAY);
		cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
		img.convertTo(img, CV_32F, 1 / 255.f);
		return img;
	}


}


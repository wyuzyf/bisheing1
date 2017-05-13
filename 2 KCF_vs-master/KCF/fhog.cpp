/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2010-2013, University of Nizhny Novgorod, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "fhog.hpp"

#ifdef HAVE_TBB
#include <tbb/tbb.h>
#include "tbb/parallel_for.h"
#include "tbb/blocked_range.h"
#endif

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

/*
// Getting feature map for the selected subimage
//
// API
// int getFeatureMaps(const IplImage * image, const int k, featureMap **map);
// INPUT
// image             - selected subimage
// k                 - size of cells
// OUTPUT
// map               - feature map
// RESULTe
// Error status
*/
int getFeatureMaps(const IplImage* image, const int k, CvLSVMFeatureMapCaskade **map)
{
	int sizeX, sizeY;
	int p; 
	int stringSize;  // X轴方向bin总数
	int height, width, numChannels;
	int i, j, kk, c, ii, jj, d;
	float *datadx, *datady;
	
	int   ch;
	float magnitude, x, y, tx, ty;
	
	IplImage *dx, *dy;  // 水平方向(X)梯度分量矩阵， 竖直方向(Y)梯度分量矩阵
	int *nearest;
	float *w;
	
	float *r;		//像素点处的梯度幅值
	int   *alfa;	// 存取各像素所处的通道ID
	
	float boundary_x[NUM_SECTOR + 1];  // 9个扇区，有9+1条边界
	float boundary_y[NUM_SECTOR + 1];
	float max, dotProd;
	int   maxi;

	height = image->height;
	width = image->width;

	numChannels = image->nChannels;  // 图像通道数

	float kernel[3] = { -1.0f, 0.0f, 1.0f };
	CvMat kernel_dx = cvMat(1, 3, CV_32F, kernel); // 水平梯度卷积算子: 行向量
	CvMat kernel_dy = cvMat(3, 1, CV_32F, kernel); // 垂直梯度卷积算子：列向量
	dx = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_32F, 3);  // 3通道， 即为 彩图
	dy = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_32F, 3);
	cvFilter2D(image, dx, &kernel_dx, cvPoint(-1, -1));	//cvPoint(-1, 0)
	cvFilter2D(image, dy, &kernel_dy, cvPoint(-1, -1));	//cvPoint(0, -1)

	sizeX = width  / k; // 水平方向上cell的个数，即单元格的数目
	sizeY = height / k;	// 垂直方向上cell的个数
	p = 3 * NUM_SECTOR;;   // 无符号 + 有符号 每个单元格的直方图通道总数  http://blog.csdn.net/ttransposition/article/details/41806601#t0
	stringSize = sizeX * p;   // X轴方向bin总数
	allocFeatureMapObject(map, sizeX, sizeY, p);	//分配内存空间

	float arg_vector; // 区间弧度值
	for(i = 0; i <= NUM_SECTOR; i++)
	{
		arg_vector    = ( (float) i ) * ( (float)(PI) / (float)(NUM_SECTOR) );
		boundary_x[i] = cosf(arg_vector);	// 区间边线（长度为1）的x坐标
		boundary_y[i] = sinf(arg_vector);	// 区间边线（长度为1）的y坐标
	}/*for(i = 0; i <= NUM_SECTOR; i++) */

	r    = (float*)malloc( sizeof(float) * (width * height) );	 // 存取各像素的梯度幅值
	alfa = (int*)malloc( sizeof(int) * (width * height * 2) );   // 存取各像素所处的通道ID,  2 表示 无符号 + 有符号 这两种情况

	for(j = 1; j < height - 1; j++)
	{
		// imageData是一个指向某内存空间的指针，该内存空间里面存放了图像实际像素点的颜色信息（这里是RGB信息)
		// widthstep图像一行像素点所占的字节数
		datadx = (float*)(dx->imageData + dx->widthStep * j);
		datady = (float*)(dy->imageData + dy->widthStep * j);
		for(i = 1; i < width - 1; i++)
		{
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 取RGB三通道中梯度幅值最大的项 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			c = 0; // RGB通道ID
			x = (datadx[i * numChannels + c]);  // 像素点处水平方向梯度
			y = (datady[i * numChannels + c]);	// 像素点处垂直方向梯度

			r[j * width + i] =sqrtf(x * x + y * y);  // 像素的梯度幅值
			for(ch = 1; ch < numChannels; ch++)
			{
				tx = (datadx[i * numChannels + ch]);
				ty = (datady[i * numChannels + ch]);
				magnitude = sqrtf(tx * tx + ty * ty);  // 梯度幅值
				if(magnitude > r[j * width + i])
				{
					r[j * width + i] = magnitude;
					c = ch;
					x = tx;
					y = ty;
				}
			}/*for(ch = 1; ch < numChannels; ch++)*/
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 取RGB三通道中梯度幅值最大的项 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 通过比较两直线内积的大小来确定该像素的梯度所处bin的位置 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			// <a,b> = ||a|| ||b|| cos(xita)，即 xita 越小，<a,b>则越大
			max  = boundary_x[0] * x + boundary_y[0] * y;
			maxi = 0;
			for (kk = 0; kk < NUM_SECTOR; kk++) 
			{
				dotProd = boundary_x[kk] * x + boundary_y[kk] * y;
				if (dotProd > max) 
				{
					max  = dotProd;
					maxi = kk;
				}
				else if (-dotProd > max)
				{
					max = -dotProd;
					maxi = kk + NUM_SECTOR;
				}
			} /* for (kk = 0; kk < NUM_SECTOR; kk++) */
			alfa[j * width * 2 + i * 2    ] = maxi % NUM_SECTOR;	// 无符号梯度反向直方图
			alfa[j * width * 2 + i * 2 + 1] = maxi;   // 有符号梯度反向直方图
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 通过比较两直线内积的大小来确定该像素的梯度所处bin的位置 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		}/*for(i = 0; i < width; i++)*/
	}/*for(j = 0; j < height; j++)*/

	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 双线性插值 对像素点位置进行线性加权 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	nearest = (int  *)malloc(sizeof(int) *  k);	// k: cells 的大小
	w       = (float*)malloc(sizeof(float) * (k * 2));
	
	for(i = 0; i < k / 2; i++)
	{
		nearest[i] = -1;
	}/*for(i = 0; i < k / 2; i++)*/

	for(i = k / 2; i < k; i++)
	{
		nearest[i] = 1;
	}/*for(i = k / 2; i < k; i++)*/

	//双线性插值 http://blog.csdn.net/zyh821351004/article/details/50730097
	// 示意图 http://pan.baidu.com/s/1i5bGZE9
	float temp;
	for(j = 0; j < k / 2; j++)
	{ 
		temp = k / 2 + j + 0.5f;
		w[j * 2] = 1.0f * temp / (float)k;
		w[j * 2 + 1] = 1 - w[j * 2];
	}/*for(j = 0; j < k / 2; j++)*/

	for(j = k / 2; j < k; j++)
	{
		temp = k - (j - k / 2 + 0.5f);
		w[j * 2] = 1.0f * temp / (float)k;
		w[j * 2 + 1] = 1 - w[j * 2];
	}/*for(j = k / 2; j < k; j++)*/

	for(i = 0; i < sizeY; i++)  //行
	{
	  for(j = 0; j < sizeX; j++) //列	
	  {
		for(ii = 0; ii < k; ii++)  //行
		{
		  for(jj = 0; jj < k; jj++)  //列
		  {
			// 过滤掉原图像矩阵的最外层
			if ((i * k + ii > 0) && (i * k + ii < height - 1) && (j * k + jj > 0) && (j * k + jj < width  - 1))
			{
			  d = (k * i + ii) * width + (j * k + jj); // 索引下标 

			  // alfa[d * 2    ] 无符号bin的ID索引
			  // alfa[d * 2 + 1] 有符号bin的ID索引
			  (*map)->map[ i * stringSize + j * (*map)->numFeatures + alfa[d * 2    ]] += r[d] * w[ii * 2] * w[jj * 2];
			  (*map)->map[ i * stringSize + j * (*map)->numFeatures + alfa[d * 2 + 1] + NUM_SECTOR] += r[d] * w[ii * 2] * w[jj * 2];

			  // nearest[0] = nearest[1] = -1, nearest[2] = nearest[3] = 1
			  if ((i + nearest[ii] >= 0) && (i + nearest[ii] <= sizeY - 1))
			  {
				(*map)->map[(i + nearest[ii]) * stringSize + j * (*map)->numFeatures + alfa[d * 2]] += r[d] * w[ii * 2 + 1] * w[jj * 2 ];
				(*map)->map[(i + nearest[ii]) * stringSize + j * (*map)->numFeatures + alfa[d * 2 + 1] + NUM_SECTOR] += r[d] * w[ii * 2 + 1] * w[jj * 2 ];
			  }

			  if ((j + nearest[jj] >= 0) && (j + nearest[jj] <= sizeX - 1))
			  {
				(*map)->map[i * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2]] += r[d] * w[ii * 2] * w[jj * 2 + 1];
				(*map)->map[i * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2 + 1] + NUM_SECTOR] += r[d] * w[ii * 2] * w[jj * 2 + 1];
			  }

			  if ((i + nearest[ii] >= 0) && (i + nearest[ii] <= sizeY - 1) && (j + nearest[jj] >= 0) && (j + nearest[jj] <= sizeX - 1))
			  {
				(*map)->map[(i + nearest[ii]) * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2]] += r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
				(*map)->map[(i + nearest[ii]) * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2 + 1] + NUM_SECTOR] += r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
			  }
			}
		  }/*for(jj = 0; jj < k; jj++)*/
		}/*for(ii = 0; ii < k; ii++)*/
	  }/*for(j = 1; j < sizeX - 1; j++)*/
	}/*for(i = 1; i < sizeY - 1; i++)*/
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 双线性插值 对像素点位置进行线性加权 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	
	cvReleaseImage(&dx);
	cvReleaseImage(&dy);

	free(w);
	free(nearest);
	
	free(r);
	free(alfa);

	return LATENT_SVM_OK;
}

/*
// Feature map Normalization and Truncation 
//
// API
// int normalizeAndTruncate(featureMap *map, const float alfa);
// INPUT
// map               - feature map
// alfa              - truncation threshold
// OUTPUT
// map               - truncated and normalized feature map
// RESULT
// Error status
*/
// 它可以通过先进行L2-norm，对结果进行截短（clipping）（即值被限制为v - 0.2v之间），然后再重新归一化得到。
int normalizeAndTruncate(CvLSVMFeatureMapCaskade *map, const float alfa)
{
	int i, j, ii;
	int sizeX, sizeY, p, pos, pp, xp, pos1, pos2;
	float *partOfNorm; // norm of C(i, j)
	float *newData;
	float valOfNorm;

	sizeX     = map->sizeX;  // X轴方向Cell的个数
	sizeY     = map->sizeY;	 // Y轴方向Cell的个数
	partOfNorm = (float *)malloc (sizeof(float) * (sizeX * sizeY));

	p  = NUM_SECTOR; // 无符号梯度直方图通道数
	xp = NUM_SECTOR * 3;	// 无符号梯度直方图通道数 + 符号梯度直方图通道数 
	pp = NUM_SECTOR * 3 * 4;  // http://blog.csdn.net/ttransposition/article/details/41806601#t0  DMP对HOG特征进行了改进

	// 计算每个单元格(cell)的特征向量（无符号梯度直方图）的内积（2范数的平方）
	for(i = 0; i < sizeX * sizeY; i++)
	{
		valOfNorm = 0.0f;
		pos = i * (map->numFeatures);
		for(j = 0; j < p; j++)
			valOfNorm += map->map[pos + j] * map->map[pos + j];
		partOfNorm[i] = valOfNorm;
	}
	
	// 过滤掉map->map矩阵最外层
	sizeX -= 2;
	sizeY -= 2;

	newData = (float *)malloc (sizeof(float) * (sizeX * sizeY * pp)); // http://blog.csdn.net/ttransposition/article/details/41806601#t0  DMP HOG特征
	
	//normalization 归一化
	for(i = 1; i <= sizeY; i++)  //行
	{
		for(j = 1; j <= sizeX; j++)  //列
		{
			pos1 = (i)* (sizeX + 2) * xp + (j)* xp;
			pos2 = (i - 1) * (sizeX)* pp + (j - 1) * pp;
			// step 1 http://pan.baidu.com/s/1i5nSpQt
			valOfNorm = sqrtf(
				partOfNorm[(i    )*(sizeX + 2) + (j    )] +
				partOfNorm[(i    )*(sizeX + 2) + (j + 1)] +
				partOfNorm[(i + 1)*(sizeX + 2) + (j    )] +
				partOfNorm[(i + 1)*(sizeX + 2) + (j + 1)]) + FLT_EPSILON;
			for(ii = 0; ii < p; ii++)
				newData[pos2 + ii] = map->map[pos1 + ii] / valOfNorm;
			for(ii = 0; ii < 2 * p; ii++)
				newData[pos2 + ii + p * 4] = map->map[pos1 + ii + p] / valOfNorm;
			// step 2
			valOfNorm = sqrtf(
				partOfNorm[(i    )*(sizeX + 2) + (j    )] +
				partOfNorm[(i    )*(sizeX + 2) + (j + 1)] +
				partOfNorm[(i - 1)*(sizeX + 2) + (j    )] +
				partOfNorm[(i - 1)*(sizeX + 2) + (j + 1)]) + FLT_EPSILON;
			for(ii = 0; ii < p; ii++)
				newData[pos2 + ii + p] = map->map[pos1 + ii] / valOfNorm;
			for(ii = 0; ii < 2 * p; ii++)
				newData[pos2 + ii + p * 6] = map->map[pos1 + ii + p] / valOfNorm;
			// step 3
			valOfNorm = sqrtf(
				partOfNorm[(i    )*(sizeX + 2) + (j    )] +
				partOfNorm[(i    )*(sizeX + 2) + (j - 1)] +
				partOfNorm[(i + 1)*(sizeX + 2) + (j    )] +
				partOfNorm[(i + 1)*(sizeX + 2) + (j - 1)]) + FLT_EPSILON;
			for(ii = 0; ii < p; ii++)
				newData[pos2 + ii + p * 2] = map->map[pos1 + ii] / valOfNorm;
			for(ii = 0; ii < 2 * p; ii++)
				newData[pos2 + ii + p * 8] = map->map[pos1 + ii + p] / valOfNorm;
			// step 4
			valOfNorm = sqrtf(
				partOfNorm[(i    )*(sizeX + 2) + (j    )] +
				partOfNorm[(i    )*(sizeX + 2) + (j - 1)] +
				partOfNorm[(i - 1)*(sizeX + 2) + (j    )] +
				partOfNorm[(i - 1)*(sizeX + 2) + (j - 1)]) + FLT_EPSILON;
			for(ii = 0; ii < p; ii++)
				newData[pos2 + ii + p * 3 ] = map->map[pos1 + ii] / valOfNorm;
			for(ii = 0; ii < 2 * p; ii++)
				newData[pos2 + ii + p * 10] = map->map[pos1 + ii + p] / valOfNorm;
		}
	}
	
	//truncation 使用 alfa = 0.2 进行截断
	for(i = 0; i < sizeX * sizeY * pp; i++)
	{
		if(newData [i] > alfa) 
			newData [i] = alfa;
	}

	//change data
	map->numFeatures = pp;
	map->sizeX = sizeX;
	map->sizeY = sizeY;

	free(map->map);
	free(partOfNorm);

	map->map = newData;

	return LATENT_SVM_OK;
}

/*
// Feature map reduction
// In each cell we reduce dimension of the feature vector
// according to original paper special procedure
//
// API
// int PCAFeatureMaps(featureMap *map)
// INPUT
// map               - feature map
// OUTPUT
// map               - feature map
// RESULT
// Error status
*/
int PCAFeatureMaps(CvLSVMFeatureMapCaskade *map)
{	
	int i, j, ii, jj, k;
	int sizeX, sizeY, p, pp, pos1, pos2; //xp, yp, 
	float *newData;
	float val;
	//float nx, ny;
	
	sizeX = map->sizeX;	//水平方向
	sizeY = map->sizeY;	//竖直方向
	p     = map->numFeatures;
	pp    = NUM_SECTOR * 3 + 4; // 31维特征向量 http://blog.csdn.net/ttransposition/article/details/41806601#t0


	//yp    = 4;
	//xp    = NUM_SECTOR;

	//nx    = 1.0f / sqrtf((float)(xp * 2));
	//ny    = 1.0f / sqrtf((float)(yp    ));

	newData = (float *)malloc (sizeof(float) * (sizeX * sizeY * pp));

	for(i = 0; i < sizeY; i++)	// 行
		for(j = 0; j < sizeX; j++)	// 列
		{
			pos1 = ((i)*sizeX + j)*p;  // p: 9*3*4 = 108 维
			pos2 = ((i)*sizeX + j)*pp; // pp: 9*3+4 = 31 维
			k = 0;
			for (jj = 0; jj < NUM_SECTOR * 2; jj++)  // 有符号梯度方向
			{
				val = 0;
				for(ii = 0; ii < 4; ii++)  // yp = 4
					val += map->map[pos1 + 4 * NUM_SECTOR + ii * NUM_SECTOR * 2 + jj]; //val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
				newData[pos2 + k] = val * 1.0f / sqrtf(4.0f);
				k++;
			}

			for (jj = 0; jj < NUM_SECTOR; jj++) // 无符号梯度方向
			{
				val = 0;
				for(ii = 0; ii < 4; ii++)
					val += map->map[pos1 + ii * NUM_SECTOR + jj];
				newData[pos2 + k] = val * 1.0f / sqrtf(4.0f);
				k++;
			}

			for(ii = 0; ii < 4; ii++)  // 无符号梯度方向
			{
				val = 0;
				//for (jj = 0; jj < 2 * NUM_SECTOR; jj++)  // original 
				//	val += map->map[pos1 + 4 * NUM_SECTOR + ii * NUM_SECTOR * 2 + jj];
				//newData[pos2 + k] = val * nx;
				for (jj = 0; jj < NUM_SECTOR; jj++)
					val += map->map[pos1 + ii * NUM_SECTOR + jj];
				newData[pos2 + k] = val * 1.0f / sqrtf((float)NUM_SECTOR);
				
				k++;
			}
		}

	// renew the data
	map->numFeatures = pp;

	free (map->map);

	map->map = newData;

	return LATENT_SVM_OK;
}

int allocFeatureMapObject(CvLSVMFeatureMapCaskade **obj, const int sizeX, const int sizeY, const int numFeatures)
{
	int i;
	(*obj) = (CvLSVMFeatureMapCaskade *)malloc(sizeof(CvLSVMFeatureMapCaskade));
	(*obj)->sizeX       = sizeX;
	(*obj)->sizeY       = sizeY;
	(*obj)->numFeatures = numFeatures;
	(*obj)->map = (float *) malloc(sizeof (float) * (sizeX * sizeY  * numFeatures));
	for(i = 0; i < sizeX * sizeY * numFeatures; i++)
		(*obj)->map[i] = 0.0f;
	return LATENT_SVM_OK;
}

int freeFeatureMapObject (CvLSVMFeatureMapCaskade **obj)
{
	if(*obj == NULL) return LATENT_SVM_MEM_NULL;
	free((*obj)->map);
	free(*obj);
	(*obj) = NULL;
	return LATENT_SVM_OK;
}

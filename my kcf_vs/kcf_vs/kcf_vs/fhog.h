
#ifndef _FHOG_H
#define _FHOG_H

#include <stdio.h>
#include <float.h>
#include <opencv2/imgproc/imgproc_c.h>

// DataType: STRUCT featureMap
// FEATURE MAP DESCRIPTION  特征图描述
//   Rectangular map (sizeX x sizeY), 
//   every cell stores feature vector (dimension = numFeatures)
//    map - matrix of feature vectors  特征向量矩阵
//          to set and get feature vectors (i,j) 
//          used formula map[(j * sizeX + i) * p + k], where
//          k - component of feature vector in cell (i, j)
typedef struct{
	int sizeX;         //ROI上X轴方向上包含的cell个数
	int sizeY;         //ROI上Y轴方向上包含的cell个数
	int numFeatures;   //every cell stores feature vector
	float *map;
}CvLSVMFeatureMapCaskade;

#define PI    CV_PI

#define EPS 0.000001

#define F_MAX FLT_MAX
#define F_MIN -FLT_MAX


// The number of elements in bin
// The number of sectors in gradient histogram building
//用9个梯度直方图来表示细胞单元
#define NUM_SECTOR 9

// The number of levels in image resize procedure图像调整大小的级别数
// We need Lambda levels to resize image twice   λ数值
#define LAMBDA 10

// Block size. Used in feature pyramid building procedure    block的边长
#define SIDE_LENGTH 8

#define VAL_OF_TRUNCATE 0.2f   //截断值？？？？

//modified from "_lsvm_error.h"
#define LATENT_SVM_OK 0
#define LATENT_SVM_MEM_NULL 2
#define DISTANCE_TRANSFORM_OK 1
#define DISTANCE_TRANSFORM_GET_INTERSECTION_ERROR -1
#define DISTANCE_TRANSFORM_ERROR -2
#define DISTANCE_TRANSFORM_EQUAL_POINTS -3
#define LATENT_SVM_GET_FEATURE_PYRAMID_FAILED -4
#define LATENT_SVM_SEARCH_OBJECT_FAILED -5
#define LATENT_SVM_FAILED_SUPERPOSITION -6
#define FILTER_OUT_OF_BOUNDARIES -7
#define LATENT_SVM_TBB_SCHEDULE_CREATION_FAILED -8
#define LATENT_SVM_TBB_NUMTHREADS_NOT_CORRECT -9
#define FFT_OK 2
#define FFT_ERROR -10
#define LSVM_PARSER_FILE_NOT_FOUND -11

/*
// Getting feature map for the selected subimage
// 从窗口中得到特征图

// INPUT
// image             - selected subimage  输入图像
// k                 - size of cells  细胞组大小 k*k
// OUTPUT
// map               - feature map  输出特征图
// RESULT
// Error status
*/
int getFeatureMaps(const IplImage* image, const int k,CvLSVMFeatureMapCaskade **map);

/*
// Feature map Normalization and Truncation
//归一化特征图和截断

// INPUT
// map               - feature map
// alfa              - truncation threshold  α截断阈值？？？
// OUTPUT
// map               - truncated and normalized feature map
// RESULT
// Error status
*/
int normalizeAndTruncate(CvLSVMFeatureMapCaskade *map,const float alfa);

/*
// Feature map reduction
// In each cell we reduce dimension of the feature vector
// according to original paper special procedure
 PCA降维
// INPUT
// map               - feature map
// OUTPUT
// map               - feature map
// RESULT
// Error status
*/

int PCAFeatureMaps(CvLSVMFeatureMapCaskade *map);

//分配特征图内存
int allocFeatureMapObject(CvLSVMFeatureMapCaskade **obj, const int sizeX, const int sizeY, const int p);

int freeFeatureMapObject(CvLSVMFeatureMapCaskade **obj);


#endif


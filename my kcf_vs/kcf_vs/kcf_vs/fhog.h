
#ifndef _FHOG_H
#define _FHOG_H

#include <stdio.h>
#include <float.h>
#include <opencv2/imgproc/imgproc_c.h>

// DataType: STRUCT featureMap
// FEATURE MAP DESCRIPTION  ����ͼ����
//   Rectangular map (sizeX x sizeY), 
//   every cell stores feature vector (dimension = numFeatures)
//    map - matrix of feature vectors  ������������
//          to set and get feature vectors (i,j) 
//          used formula map[(j * sizeX + i) * p + k], where
//          k - component of feature vector in cell (i, j)
typedef struct{
	int sizeX;         //ROI��X�᷽���ϰ�����cell����
	int sizeY;         //ROI��Y�᷽���ϰ�����cell����
	int numFeatures;   //every cell stores feature vector
	float *map;
}CvLSVMFeatureMapCaskade;

#define PI    CV_PI

#define EPS 0.000001

#define F_MAX FLT_MAX
#define F_MIN -FLT_MAX


// The number of elements in bin
// The number of sectors in gradient histogram building
//��9���ݶ�ֱ��ͼ����ʾϸ����Ԫ
#define NUM_SECTOR 9

// The number of levels in image resize procedureͼ�������С�ļ�����
// We need Lambda levels to resize image twice   ����ֵ
#define LAMBDA 10

// Block size. Used in feature pyramid building procedure    block�ı߳�
#define SIDE_LENGTH 8

#define VAL_OF_TRUNCATE 0.2f   //�ض�ֵ��������

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
// �Ӵ����еõ�����ͼ

// INPUT
// image             - selected subimage  ����ͼ��
// k                 - size of cells  ϸ�����С k*k
// OUTPUT
// map               - feature map  �������ͼ
// RESULT
// Error status
*/
int getFeatureMaps(const IplImage* image, const int k,CvLSVMFeatureMapCaskade **map);

/*
// Feature map Normalization and Truncation
//��һ������ͼ�ͽض�

// INPUT
// map               - feature map
// alfa              - truncation threshold  ���ض���ֵ������
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
 PCA��ά
// INPUT
// map               - feature map
// OUTPUT
// map               - feature map
// RESULT
// Error status
*/

int PCAFeatureMaps(CvLSVMFeatureMapCaskade *map);

//��������ͼ�ڴ�
int allocFeatureMapObject(CvLSVMFeatureMapCaskade **obj, const int sizeX, const int sizeY, const int p);

int freeFeatureMapObject(CvLSVMFeatureMapCaskade **obj);


#endif

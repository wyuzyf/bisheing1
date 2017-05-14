/* @file 
	与粒子滤波跟踪有关的定义
   @author  zhao lu
   @version 20170414
*/

#ifndef PARTICLES_H
#define PARTICLES_H

#include "observation.h"

using namespace cv;



/************************** Definitions **************************/

//在重采样过程中，过度（预测）模型的标准高斯参数
#define TRANS_X_STD 1.0
#define TRANS_Y_STD 0.5
#define TRANS_S_STD 0.001

//二阶自回归模型参数
#define A1 2.0
#define A2 -1.0
#define B0 1.0000


/******************************* Structures **********************************/

/**
	粒子是系统的状态变量，粒子的集合本质上系统后验概率的离散化
*/
 struct particle{
	float x;          /**< current x coordinate */
	float y;          /**< current y coordinate */
	float s;          /**< scale */
	
	float xp;         /**< previous x coordinate */
	float yp;         /**< previous y coordinate */
	float sp;         /**< previous scale */
	
	float x0;         /**< original x coordinate */
	float y0;         /**< original y coordinate */
	int width;        /**< original width of region described by particle */
	int height;       /**< original height of region described by particle */

	histogram* histo;   //选取目标区域的直方图作为参考直方图
	float w;   //粒子权重
};

/**************************** Function Prototypes ****************************/

/**
	通过高斯采样来创建粒子的初始分布

	@param regions 要对其进行采样的目标区域数组
	@param histos array of histograms describing regions in \a regions
	@param n the number of regions in \a regions
	@param p the total number of particles to be assigned

	@return Returns an array of \a p particles sampled from around regions in
	\a regions
*/
particle* init_distribution(CvRect* regions, histogram** histos, int n, int p);

/**
	为给定粒子建立采样的过度模型

	@param p a particle to be transitioned
	@param w video frame width
	@param h video frame height
	@param rng a random number generator from which to sample

	@return 返回新的粒子集
*/
particle transition(particle p,int w, int h, RNG* rng);


/**
	Normalizes particle weights so they sum to 1

	@param particles an array of particles whose weights are to be normalized
	@param n the number of particles in \a particles
*/
void normalize_weights(particle* particles,int n);

/**
	重采样一组加权粒子用来产生一组为加权的粒子

	@param particles an old set of weighted particles whose weights have been
	normalized with normalize_weights()
	@param n the number of particles in \a particles

	@return Returns a new set of unweighted particles sampled from \a particles
*/
particle* resample(particle* particles,int n);

/**
	Compare two particles based on weight.  For use in qsort.

	@param p1 pointer to a particle
	@param p2 pointer to a particle

	@return Returns -1 if the \a p1 has lower weight than \a p2, 1 if \a p1
	has higher weight than \a p2, and 0 if their weights are equal.
*/
int particle_cmp(const void* p1,const  void* p2);

/**
	显示指定的粒子矩形区域

	@param img the image on which to display the particle
	@param p the particle to be displayed
	@param color the color in which \a p is to be displayed
*/
void display_particle(IplImage* img, particle p,CvScalar color);




#endif


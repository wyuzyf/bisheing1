/*
	Functions for object tracking with a particle filter

	@author zhao lu
	@version 20170414
*/

#include "defs.h"
#include "utils.h"
#include "particles.h"


/*************************** Function Definitions ****************************/

//初始化粒子集
particle* init_distribution(CvRect* regions, histogram** histos, int n, int p)
{
	particle* particles;
	int np;   
	float x, y;
	int i, j, width, height, k = 0;

	particles = (particle*)malloc(p * sizeof(particle));
	np = p / n;   //粒子密度=粒子总数 / 感兴趣区域n

	//在n个regions的中心创建粒子
	for (i = 0; i < n; i++)
	{
		//找到矩形的中心坐标，regions[i].x是矩形左上角坐标
		width = regions[i].width;
		height = regions[i].height;
		x = regions[i].x + width / 2;
		y = regions[i].y + height / 2;

		for (j = 0; j < np; j++)
		{
			//原始 = 预测 =当前 ，original是不变的，它是选取的目标特征
			particles[k].x0 = particles[k].xp = particles[k].x = x;
			particles[k].y0 = particles[k].yp = particles[k].y = y;
			particles[k].sp = particles[k].s = 1.0;
			particles[k].width = width;
			particles[k].height = height;
			particles[k].histo = histos[i];
			particles[k++].w = 0;
		}
	}

	//确保在每个矩形中创建了准确的p个粒子
	i = 0;
	while (k < p)
	{
		width = regions[i].width;
		height = regions[i].height;
		x = regions[i].x + width / 2;
		y = regions[i].y + height / 2;
		particles[k].x0 = particles[k].xp = particles[k].x = x;
		particles[k].y0 = particles[k].yp = particles[k].y = y;
		particles[k].sp = particles[k].s = 1.0;
		particles[k].width = width;
		particles[k].height = height;
		particles[k].histo = histos[i];
		particles[k++].w = 0;
		i = (i + 1) % n;
	}

	return particles;   //返回一个结构体

}

//根据二阶自回归，构造状态转移模型，根据模型得到新的粒子集
particle transition(particle p, int w, int h, RNG rng)
{
	float x, y, s;
	particle pn;
	
	/* sample new state using second-order autoregressive dynamics */
	x = A1 * (p.x - p.x0) + A2*(p.xp - p.x0) + B0 * rng.gaussian(TRANS_X_STD) + p.x0;
	pn.x = MAX(0.0,MIN((float)w - 1.0, x));

	y = A1 * (p.y - p.y0) + A2*(p.yp - p.y0) + B0 * rng.gaussian(TRANS_Y_STD) + p.y0;
	pn.x = MAX(0.0, MIN((float)w - 1.0, y));

	s = A1 * (p.s - 1.0) + A2 * (p.sp - 1.0) + B0 * rng.gaussian(TRANS_S_STD) + 1.0;
	pn.s = MAX(0.1,s);
	
	//上一个粒子集当前的参数 = 新粒子集的预测
	pn.xp = p.x;
	pn.yp = p.y;
	pn.sp = p.s;
	pn.x0 = p.x0;
	pn.y0 = p.y0;
	pn.width = p.width;
	pn.height = p.height;
	pn.histo = p.histo;
	pn.w = 0;

	return pn;      //返回新的粒子集
}

//归一化粒子的权值
void normalize_weights(particle* particles, int n)
{
	float sum = 0;
	int i;

	for (i = 0; i < n; i++)
		sum += particles[i].w;
	for (i = 0; i < n; i++)
		particles[i].w /= sum;
}


//根据已有权值的粒子集 (重采样) 得到新的无权值的粒子集
particle* resample(particle* particles, int n)
{
	particle* new_particles;
	int i, j, np, k = 0;

	qsort(particles, n, sizeof(particle), particle_cmp);
	new_particles = (particle*)malloc(n * sizeof(particle));
	for (i = 0; i < n; i++)
	{
		//四舍五入，np越大，说明权值越高,然后在此权值粒子的附近取粒子
		np = cvRound(particles[i].w * n);   
		for (j = 0; j < np; j++)  //保证了权值是0的粒子肯定被删掉
		{
			new_particles[k++] = particles[i];     //?????
			if (k == n)
				goto exit;
		}
	}
	while (k < n)
		new_particles[k++] = particles[0];

exit:
	return new_particles;

}

//比较美个粒子的权值
int particle_cmp(const void* p1, const void* p2)
{
	particle* _p1 = (particle*)p1;
	particle* _p2 = (particle*)p2;

	if (_p1->w > _p2->w)
		return -1;
	if (_p1->w < _p2->w)
		return 1;
	return 0;
}

//region是个矩形，用粒子表示的
void dispaly_particle(IplImage* img, particle p, CvScalar color)
{
	int x0, y0, x1, y1;
	x0 = cvRound(p.x - 0.5 * p.s * p.width);
	y0 = cvRound(p.y - 0.5 * p.s * p.height);
	x1 = x0 + cvRound(p.s * p.width);
	y1 = y0 + cvRound(p.s * p.height);

	cvRectangle(img, cvPoint(x0,y0), cvPoint(x1,y1),color,1,8,0);

}


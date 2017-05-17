
#include "pf.h"


using namespace cv;

Mat hsv;    //hsv色彩空间的输入图像
Mat roiImage;      //目标区域
Mat Image;        //选取的目标图像
MatND roiHist;    //计算出来的目标区域直方图

Rect selection;     //感兴趣的区域大小


//快速排序，排序粒子权重值大小
int particle_cmp(const void *p1, const void *p2) //俩个参数是无类型指针
{
	PARTICLE* _p1 = (PARTICLE*)p1;    //强制转换指针类型
	PARTICLE* _p2 = (PARTICLE*)p2;

	if (_p1->weight < _p2->weight)
		return 1;    //按照权重降序排序？？？？
	if (_p1->weight > _p2->weight)
		return -1;
	return 0;
}

//粒子滤波算法实现
void particle_filter(void)
{
	PARTICLE *pParticles = particles;

	cvtColor(Image, hsv, CV_BGR2HSV);   //将BGR转换成HSV模型
	Mat roiImage(hsv, selection);       //目标区域的直方图

	if (nFrameNum == 1)     //是否为第一帧
	{
		//step1:提取目标区域特征
		calcHist(&roiImage,1,channels,Mat(),roiHist,3,histSize,ranges);
		normalize(roiHist,roiHist);   //归一化直方图
		    //因为全局变量在用到的时候会更新为当前值，所以俩个参数一样不会影响
		
		//step2:初始化particles
		pParticles = particles;   //数组
		for (int i = 0; i < Particle_num; i++)
		{
			//因为是初始化，所以当前=原始=预测
			pParticles->x = selection.x + 0.5* selection.width;
			pParticles->y = selection.y + 0.5* selection.height;

			pParticles->xPre = pParticles->x;
			pParticles->yPre = pParticles->y;

			pParticles->xOri = pParticles->x;
			pParticles->yOri = pParticles->y;

			pParticles->rect = selection;
			pParticles->scale = 1.0;
			pParticles->scalePre = 1.0;
			pParticles->hist = roiHist;
			pParticles->weight = 0;
			pParticles++;

		}
	}

	else    //不是第一帧
	{
		pParticles = particles;
		RNG rng;  //均匀分布随机数和高斯分布随机数
		for (int i = 0; i < Particle_num; i++)
		{
			//step3:求particles的下一组数据？？？？
			//状态转移阶段，即根据上一帧中粒子的状态（x,y,w,h）来估计本帧中各个粒子的状态
			
			//目前还没完全理解原理，所以下面的注释都是猜想，不一定正确
			
			double x, y, s;

			//当前=预测,把上一帧当做当前一帧的预测，然后根据预测计算当前一帧，和原始的ori没关系了
			pParticles->xPre = pParticles->x; 
			pParticles->yPre = pParticles->y;
			pParticles->scale = pParticles->scale;

			//计算粒子的坐标
			x = A1 * (pParticles->x - pParticles->xOri) + A2 * (pParticles->xPre - pParticles->xOri)
				+ B0 * rng.gaussian(SIGMA_X) + pParticles->xOri;   
			//这个pParticles->xPre = pParticles->x，那前俩个式子除了正负可能不一样之外，还有什么区别么？？
			pParticles->x = std::max(0.0,std::min(x,Image.cols - 1.0));  

			
			y = A1 * (pParticles->y - pParticles->yOri) + A2 * (pParticles->yPre - pParticles->yOri)
				+ B0 * rng.gaussian(SIGMA_Y) + pParticles->yOri;
			pParticles->y = std::max(0.0,std::min(y,Image.rows - 1.0));

			s = A1 * (pParticles->scale - 1.0) + A2 * (pParticles->scalePre - 1.0)
				+ B0 * rng.gaussian(SIGMA_SCALE) + 1.0;
			pParticles->scale = std::max(0.1, std::min(s,3.0));

			//计算粒子所在矩形的参数
			pParticles->rect.x = max(0,min(cvRound(pParticles->x - 0.5 * pParticles->rect.width * pParticles->scale),Image.cols-1));
			                                          //进行四舍五入，返回一个整形数
			pParticles->rect.y = max(0, min(cvRound(pParticles->y - 0.5 * pParticles->rect.height * pParticles->scale), Image.rows - 1));
			
			pParticles->rect.width = min(cvRound(pParticles->rect.width * pParticles->scale),Image.cols - pParticles->rect.x);
			pParticles->rect.height = min(cvRound(pParticles->rect.height * pParticles->scale), Image.rows - pParticles->rect.y);


			//step 4:求当前的粒子集特征直方图
			Mat imgParticle(Image,pParticles->rect);
			calcHist(&imgParticle, 1, channels, Mat(),pParticles->hist,3,histSize,ranges);
			normalize(pParticles->hist,pParticles->hist);   //归一化

			//step 5:特征的对比，更新particle权重
			pParticles->weight = compareHist(roiHist,pParticles->hist,CV_COMP_INTERSECT);
			                              //计算相似度的方法是CV_COMP_INTERSECT,输出值大小为0-1

			pParticles++;   //计算Particle_num次
		}

		//step 6:归一化权重
		double sum = 0.0;
		int i;
		
		//先求和，然后归一化
		pParticles = particles;
		for (i = 0; i < Particle_num; i++)
		{
			sum += pParticles->weight;
			pParticles++;
		}

		//pParticles = particles;  我认为这句是多余的，因为前面已经赋值了，且这个期间没有改变过
		for (i = 0; i < Particle_num; i++)
		{
			pParticles->weight /= sum;
			pParticles++;
		}


		//step 7:resamle重采样，根据粒子的权重的后验概率密度重采样
		pParticles = particles;
		PARTICLE newParticles[Particle_num];   //新的粒子集
		int np;    //后验概率密度
		int k = 0;

		qsort(pParticles,Particle_num,sizeof(PARTICLE),&particle_cmp); //particle_cmp应该是个回调函数

		//不是很理解
		for (int i = 0; i < Particle_num; i++)
		{
			np = cvRound(particles[i].weight * Particle_num);     //可能是粒子值，值越大说明权值越大
			for (int j = 0; j < np; j++)
			{
				newParticles[k++] = particles[i];
				if (k == Particle_num)
					goto EXITOUT;
			}
		}
		while (k<Particle_num)
		{
			newParticles[k++] = particles[0];
		}
	EXITOUT:
		for (int i = 0; i < Particle_num; i++)
		{
			particles[i] = newParticles[i];
		}
	}  //end else

	//得到新的粒子集后对粒子的权重重新排序
	qsort(pParticles,Particle_num,sizeof(PARTICLE),&particle_cmp); 

	//step 8:计算粒子的期望，作为跟踪结果
	Rect_<double> recTrackingTemp(0.0,0.0,0.0,0.0);
	pParticles = particles;
	for (int i = 0; i < Particle_num; i++)
	{
		recTrackingTemp.x += pParticles->rect.x * pParticles->weight;
		recTrackingTemp.y += pParticles->rect.y * pParticles->weight;
		recTrackingTemp.width += pParticles->rect.width * pParticles->weight;
		recTrackingTemp.height += pParticles->rect.height * pParticles->weight;

		pParticles++;
	}

	Rect recTracking(recTrackingTemp);  //跟踪结果

	//显示各粒子的运动
	for (int i = 0; i < Particle_num; i++)
	{
		rectangle(Image,particles[i].rect,Scalar(255,0,0));  //利用对角线定点画一个矩形
	}
	//显示跟踪结果
	rectangle(Image, recTracking, Scalar(0,0,255),3);    //传入矩形参数画矩形

	
}
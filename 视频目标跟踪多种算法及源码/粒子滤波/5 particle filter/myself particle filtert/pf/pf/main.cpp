/*
	Perform single object tracking with particle filtering

	@author zhao lu
	@version 20170414
*/

#include "defs.h"
#include "utils.h"
#include "particles.h"
#include "observation.h"

//#include<Windows.h>


using namespace cv;



/* command line options ??????*/
#define OPTIONS ":p:oah"

//default number of particles
#define PARTICLES 100

/* default basename and extension of exported frames ????*/
#define EXPORT_BASE "./frames/frame_"
#define EXPORT_EXTN ".png"

//导出的最大帧数
#define MAX_FRAMES 2048

/********************************* Structures ********************************/

//跟踪目标数
#define MAX_OBJECTS 1

typedef struct params
{
	CvPoint loc1[MAX_OBJECTS];
	CvPoint loc2[MAX_OBJECTS];
	IplImage* objects[MAX_OBJECTS];
	char* win_name;   //窗口名称
	IplImage* orig_img;   //原始图
	IplImage* cur_img;    //当前图
	int n;
}params;

/***************************** Function Prototypes ***************************/
void usage(char*);    //用户函数
int get_regions(IplImage*,CvRect**);   //手动选取目标	区域
void mouse(int,int,int,int,void*);     //鼠标响应函数
histogram** compute_ref_histos(IplImage*, CvRect*, int);    //计算参考（目标）直方图
int export_ref_histos(histogram**, int);   //导出直方图

/********************************** Globals **********************************/
char* pname;    //程序name
char* vid_file = "soccer.avi";     //input video file name
int num_particles = PARTICLES;      //number of particles
int show_all = 1;    //TRUE to display all particles
int export = TRUE;    //成功的导出目标序列






/************************** Function Definitions *****************************/

//打印程序信息
void usage(char* name)
{
	fprintf(stderr, "%s: track a single object using particle filtering\n\n",
		name);
	fprintf(stderr, "Usage: %s [options] <vid_file>\n\n", name);
	fprintf(stderr, "Arguments:\n");
	fprintf(stderr, "  <vid_file>          A clip of video in which " \
		"to track an object\n");
	fprintf(stderr, "\nOptions:\n");
	fprintf(stderr, "  -h                  Display this message and exit\n");
	fprintf(stderr, "  -a                  Display all particles, not just " \
		"the most likely\n");
	fprintf(stderr, "  -o                  Output tracking sequence frames as " \
		"%s*%s\n", EXPORT_BASE, EXPORT_EXTN);
	fprintf(stderr, "  -p <particles>      Number of particles (default %d)\n",
		PARTICLES);
}

/**
	用户选取目标区域

	@param frame the frame of video in which objects are to be selected
	@param regions a pointer to an array to be filled with rectangles
	defining object regions

	return 返回被user选择的目标数
*/
int get_regions(IplImage* frame, CvRect** regions)
{
	char* win_name = "First frame";
	params p;
	CvRect* r;
	int i, x1, y1, x2, y2, w, h;

	//用户通过鼠标来定义目标区域
	p.win_name = win_name;
	p.orig_img = (IplImage*)cvClone(frame);
	p.cur_img = NULL;
	p.n = 0;
	cvNamedWindow(win_name,1);
	cvShowImage(win_name, frame); 
	cvSetMouseCallback(win_name,&mouse,&p);
	cvWaitKey(0);
	cvDestroyWindow(win_name);
	cvReleaseImage(&(p.orig_img));
	if (p.cur_img)
		cvReleaseImage(&(p.cur_img));

	//提取regions的参数
	if (p.n = 0)
	{
		*regions = NULL;
		return 0;
	}
	r = (CvRect*)malloc(p.n * sizeof(CvRect));
	for (i = 0; i < p.n; i++)
	{
		x1 = MIN(p.loc1[i].x, p.loc2[i].x);
		x2 = MAX(p.loc1[i].x, p.loc2[i].x);
		y1 = MIN(p.loc1[i].y, p.loc2[i].y);
		y2 = MAX(p.loc1[i].y, p.loc2[i].y);
		w = x2 - x1;
		h = y2 - y1;

		/* ensure odd奇数 width and height */
		w = (w % 2) ? w : w + 1;
		h = (h % 2) ? h : h + 1;
		r[i] = cvRect(x1, y1, w, h);
	}
	*regions = r;
	return p.n;
}

/*
	Mouse callback function that allows user to specify the initial object
	regions.  Parameters are as specified in OpenCV documentation.
*/
void mouse(int event, int x, int y, int flags, void* param)
{
	params* p = (params*)param;
	CvPoint* loc;
	int n;
	IplImage* tmp;
	static int pressed = FALSE;

	/* on left button press, remember first corner of rectangle around object */
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		n = p->n;
		if (n == MAX_OBJECTS)
			return;
		loc = p->loc1;
		loc[n].x = x;
		loc[n].y = y;
		pressed = TRUE;
	}

	/* on left button up, finalize the rectangle and draw it in black */
	else if (event == CV_EVENT_LBUTTONUP)
	{
		n = p->n;
		if (n == MAX_OBJECTS)
			return;
		loc = p->loc2;
		loc[n].x = x;
		loc[n].y = y;
		cvReleaseImage(&(p->cur_img));
		p->cur_img = NULL;
		cvRectangle(p->orig_img, p->loc1[n], loc[n], CV_RGB(0, 0, 0), 1, 8, 0);
		cvShowImage(p->win_name, p->orig_img);
		pressed = FALSE;
		p->n++;
	}

	/* on mouse move with left button down, draw rectangle as defined in white */
	else if (event == CV_EVENT_MOUSEMOVE  &&  flags & CV_EVENT_FLAG_LBUTTON)
	{
		n = p->n;
		if (n == MAX_OBJECTS)
			return;
		tmp = (IplImage*)cvClone(p->orig_img);
		loc = p->loc1;
		cvRectangle(tmp, loc[n], cvPoint(x, y), CV_RGB(255, 255, 255), 1, 8, 0);
		cvShowImage(p->win_name, tmp);
		if (p->cur_img)
			cvReleaseImage(&(p->cur_img));
		p->cur_img = tmp;
	}
}



/*
	Computes a reference histogram for each of the object regions defined by
	the user

	@param frame video frame in which to compute histograms; should have been
	converted to hsv using bgr2hsv in observation.h
	@param regions regions of \a frame over which histograms should be computed
	@param n number of regions in \a regions
	@param export if TRUE, object region images are exported

	@return Returns an \a n element array of normalized histograms corresponding
	to regions of \a frame specified in \a regions.
*/
histogram** compute_ref_histos(IplImage* frame, CvRect* regions, int n)
{
	histogram** histos = (histogram**)malloc(n * sizeof(histogram*));
	IplImage* tmp;
	int i;

	/* extract each region from frame and compute its histogram */
	for (i = 0; i < n; i++)
	{
		cvSetImageROI(frame, regions[i]);
		tmp = cvCreateImage(cvGetSize(frame), IPL_DEPTH_32F, 3);
		cvCopy(frame, tmp, NULL);
		cvResetImageROI(frame);
		histos[i] = calc_histogram(&tmp, 1);
		normalize_histogram(histos[i]);
		cvReleaseImage(&tmp);
	}

	return histos;
}

/*
	Exports reference histograms to file

	@param ref_histos array of reference histograms
	@param n number of histograms

	@return Returns 1 on success or 0 on failure
*/
int export_ref_histos(histogram** ref_histos, int n)
{
	char name[32];
	char num[3];
	FILE* file;
	int i;

	for (i = 0; i < n; i++)
	{
		sprintf_s(num, (char*)3, "%02d", i);
		strcpy_s(name, "hist_");
		strcat_s(name, num);
		strcat_s(name, ".dat");
		if (!export_histogram(ref_histos[i], name))
			return 0;
	}

	return 1;
}


/*********************************** Main ************************************/
int main(int argc, char **argv)
{
	RNG rng;
	IplImage* frame, *hsv_frame, *frames[MAX_FRAMES];
	IplImage** hsv_ref_imgs;
	histogram** ref_histos;
	CvCapture* video;
	particle* particles, *new_particles;
	CvScalar color;
	CvRect* regions;
	int num_objects = 0;
	float s;
	int i, j, k, w, h, x, y;

	video = cvCaptureFromFile(vid_file);

	if (!video)
		fatal_error("could not open video file %s", vid_file);

	i = 0;
	while (frame = cvQueryFrame(video)) //从摄像头或文件中提取一帧
	{
		hsv_frame = bgr2hsv(frame);
		frames[i] = (IplImage*)cvClone(frame);

		//（1）在第一帧的时候允许用户去选取目标
		if (i == 0)
		{
			w = frame->width;
			h = frame->height;
			fprintf(stderr, "Select object region to track\n");
			while (num_objects == 0)
			{
				num_objects = get_regions(frame, &regions);
				if (num_objects == 0)
					fprintf(stderr, "Please select a object\n");
			}

			//（2）计算参考直方图
			ref_histos = compute_ref_histos(hsv_frame, regions, num_objects);
			if (export)
				export_ref_histos(ref_histos, num_objects);
			//(3)初始化粒子
			particles = init_distribution(regions, ref_histos, num_objects, num_particles);
		}

		else    //不是第一帧
		{
			//（4）对每个粒子进行预测和计算
			for (j = 0; j < num_particles; j++)
			{
				particles[j] = transition(particles[j], w, h, &rng);
				s = particles[j].s;
				//计算权值
				particles[j].w = likelihood(hsv_frame,
					cvRound(particles[j].y),
					cvRound(particles[j].x),
					cvRound(particles[j].width * s),
					cvRound(particles[j].height * s),
					particles[j].histo);
			}

			//(5)归一化权值
			normalize_weights(particles, num_particles);

			//（6）重采样不带权值的粒子
			new_particles = resample(particles, num_particles);
			free(particles);
			particles = new_particles;

			//（7）display all particles if requested
			qsort(particles, num_particles, sizeof(particle), particle_cmp);
			if (show_all)
			for (j = num_particles - 1; j > 0; j--)
			{
				color = CV_RGB(255, 255, 0);
				display_particle(frames[i], particles[j], color);
			}

			//(8) 显示相似度最高的粒子
			color = CV_RGB(255, 0, 0);
			display_particle(frames[i], particles[0], color);

			cvNamedWindow("Video", 1);
			cvShowImage("Video", frames[i]);
			if (cvWaitKey(5) == 27)
				break;

			cvReleaseImage(&hsv_frame);
			i++;
		}
		cvReleaseCapture(&video);
	}
}

#include "opencvtest.h"

using namespace std;
using namespace cv;

// HOGDescriptor visual_imagealizer
// adapted for arbitrary size of feature sets and training images
Mat get_hogdescriptor_visual_image(Mat& origImg,
	vector<float>& descriptorValues,//hog特征向量
	Size winSize,//图片窗口大小
	Size cellSize,             
	int scaleFactor,//缩放背景图像的比例
	double viz_factor)//缩放hog特征的线长比例
{   
	Mat visual_image;//最后可视化的图像大小
	resize(origImg, visual_image, Size(origImg.cols*scaleFactor, origImg.rows*scaleFactor));

	int gradientBinSize = 9;
	// dividing 180° into 9 bins, how large (in rad) is one bin?
	float radRangeForOneBin = 3.14/(float)gradientBinSize; //pi=3.14对应180°

	// prepare data structure: 9 orientation / gradient strenghts for each cell
	int cells_in_x_dir = winSize.width / cellSize.width;//x方向上的cell个数
	int cells_in_y_dir = winSize.height / cellSize.height;//y方向上的cell个数
	int totalnrofcells = cells_in_x_dir * cells_in_y_dir;//cell的总个数
	//注意此处三维数组的定义格式
	//int ***b;
	//int a[2][3][4];
	//int (*b)[3][4] = a;
	//gradientStrengths[cells_in_y_dir][cells_in_x_dir][9]
	float*** gradientStrengths = new float**[cells_in_y_dir];
	int** cellUpdateCounter   = new int*[cells_in_y_dir];
	for (int y=0; y<cells_in_y_dir; y++)
	{
		gradientStrengths[y] = new float*[cells_in_x_dir];
		cellUpdateCounter[y] = new int[cells_in_x_dir];
		for (int x=0; x<cells_in_x_dir; x++)
		{
			gradientStrengths[y][x] = new float[gradientBinSize];
			cellUpdateCounter[y][x] = 0;

			for (int bin=0; bin<gradientBinSize; bin++)
				gradientStrengths[y][x][bin] = 0.0;//把每个cell的9个bin对应的梯度强度都初始化为0
		}
	}

	// nr of blocks = nr of cells - 1
	// since there is a new block on each cell (overlapping blocks!) but the last one
	//相当于blockstride = (8,8)
	int blocks_in_x_dir = cells_in_x_dir - 1;
	int blocks_in_y_dir = cells_in_y_dir - 1;

	// compute gradient strengths per cell
	int descriptorDataIdx = 0;
	int cellx = 0;
	int celly = 0;

	for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
	{
		for (int blocky=0; blocky<blocks_in_y_dir; blocky++)            
		{
			// 4 cells per block ...
			for (int cellNr=0; cellNr<4; cellNr++)
			{
				// compute corresponding cell nr
				int cellx = blockx;
				int celly = blocky;
				if (cellNr==1) celly++;
				if (cellNr==2) cellx++;
				if (cellNr==3)
				{
					cellx++;
					celly++;
				}

				for (int bin=0; bin<gradientBinSize; bin++)
				{
					float gradientStrength = descriptorValues[ descriptorDataIdx ];
					descriptorDataIdx++;

					gradientStrengths[celly][cellx][bin] += gradientStrength;//因为C是按行存储

				} // for (all bins)


				// note: overlapping blocks lead to multiple updates of this sum!
				// we therefore keep track how often a cell was updated,
				// to compute average gradient strengths
				cellUpdateCounter[celly][cellx]++;//由于block之间有重叠，所以要记录哪些cell被多次计算了

			} // for (all cells)


		} // for (all block x pos)
	} // for (all block y pos)


	// compute average gradient strengths
	for (int celly=0; celly<cells_in_y_dir; celly++)
	{
		for (int cellx=0; cellx<cells_in_x_dir; cellx++)
		{

			float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];

			// compute average gradient strenghts for each gradient bin direction
			for (int bin=0; bin<gradientBinSize; bin++)
			{
				gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
			}
		}
	}

	
	cout << "winSize = " << winSize << endl;
	cout << "cellSize = " << cellSize << endl;
	cout << "blockSize = " << cellSize*2<< endl;
	cout << "blockNum = " << blocks_in_x_dir<<"×"<<blocks_in_y_dir << endl;
	cout << "descriptorDataIdx = " << descriptorDataIdx << endl;

	// draw cells
	for (int celly=0; celly<cells_in_y_dir; celly++)
	{
		for (int cellx=0; cellx<cells_in_x_dir; cellx++)
		{
			int drawX = cellx * cellSize.width;
			int drawY = celly * cellSize.height;

			int mx = drawX + cellSize.width/2;
			int my = drawY + cellSize.height/2;

			rectangle(visual_image,
				Point(drawX*scaleFactor,drawY*scaleFactor),
				Point((drawX+cellSize.width)*scaleFactor,
				(drawY+cellSize.height)*scaleFactor),
				CV_RGB(0,0,0),//cell框线的颜色
				1);

			// draw in each cell all 9 gradient strengths
			for (int bin=0; bin<gradientBinSize; bin++)
			{
				float currentGradStrength = gradientStrengths[celly][cellx][bin];

				// no line to draw?
				if (currentGradStrength==0)
					continue;

				float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;//取每个bin里的中间值，如10°,30°,...,170°.

				float dirVecX = cos( currRad );
				float dirVecY = sin( currRad );
				float maxVecLen = cellSize.width/2;
				float scale = viz_factor; // just a visual_imagealization scale,
				// to see the lines better

				// compute line coordinates
				float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
				float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
				float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
				float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;

				// draw gradient visual_imagealization
				line(visual_image,
					Point(x1*scaleFactor,y1*scaleFactor),
					Point(x2*scaleFactor,y2*scaleFactor),
					CV_RGB(255,255,255),//HOG可视化的cell的颜色
					1);

			} // for (all bins)

		} // for (cellx)
	} // for (celly)


	// don't forget to free memory allocated by helper data structures!
	for (int y=0; y<cells_in_y_dir; y++)
	{
		for (int x=0; x<cells_in_x_dir; x++)
		{
			delete[] gradientStrengths[y][x];            
		}
		delete[] gradientStrengths[y];
		delete[] cellUpdateCounter[y];
	}
	delete[] gradientStrengths;
	delete[] cellUpdateCounter;

	return visual_image;//返回最终的HOG可视化图像

}


int main()
{
	
	HOGDescriptor hog;//使用的是默认的hog参数
	/*
	HOGDescriptor(Size win_size=Size(64, 128), Size block_size=Size(16, 16), Size block_stride=Size(8, 8), 
	Size cell_size=Size(8, 8), int nbins=9, double win_sigma=DEFAULT_WIN_SIGMA(DEFAULT_WIN_SIGMA=-1), 
	double threshold_L2hys=0.2, bool gamma_correction=true, int nlevels=DEFAULT_NLEVELS)

	Parameters:	
	win_size – Detection window size. Align to block size and block stride.
	block_size – Block size in pixels. Align to cell size. Only (16,16) is supported for now.
	block_stride – Block stride. It must be a multiple of cell size.
	cell_size – Cell size. Only (8, 8) is supported for now.
	nbins – Number of bins. Only 9 bins per cell are supported for now.
	win_sigma – Gaussian smoothing window parameter.
	threshold_L2hys – L2-Hys normalization method shrinkage.
	gamma_correction – Flag to specify whether the gamma correction preprocessing is required or not.
	nlevels – Maximum number of detection window increases.
	*/
	//对于128*80的图片，blockstride = 8,15*9的block，2*2*9*15*9 = 4860

	int width = 80;
	int height = 128;
	hog.winSize=Size(width,height);
	vector<float> des;//HOG特征向量
	Mat src = imread("objimg.jpg");
	Mat dst ;
	resize(src,dst,Size(width,height));//规范图像尺寸
	imshow("src",src);
	hog.compute(dst,des);//计算hog特征
	Mat background = Mat::zeros(Size(width,height),CV_8UC1);//设置黑色背景图，因为要用白色绘制hog特征

	Mat d = get_hogdescriptor_visual_image(background,des,hog.winSize,hog.cellSize,3,2.5);
	imshow("dst",d);
	imwrite("hogvisualize.jpg",d);
	waitKey();

	 return 0;
}

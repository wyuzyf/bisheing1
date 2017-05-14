#include <mex.h>
#include <matrix.h>

int sub2ind(int y, int x, int height)
{
	return x * height + y;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs,
                 const mxArray *prhs[])
{
	mxArray *objWidth, *objHeight, *objIntegralImage;
	double *rect, *integralImage;
	int x1, y1, x2, y2;
	int width, height;
    int nRect, iRect;
	double *val;
	int i, j;
	
	
	// get fields of the first argument <obj>
	objWidth  = mxGetField(prhs[0], 0, "width");
	objHeight = mxGetField(prhs[0], 0, "height");
	objIntegralImage = mxGetField(prhs[0], 0, "integralImage");
	
	
	// convert mexArray to their actual data types
	width  = (int)mxGetScalar(objWidth);
	height = (int)mxGetScalar(objHeight);
	integralImage = mxGetPr(objIntegralImage);

	
	if (nrhs == 1) {
        // allocate output
        plhs[0] = mxCreateDoubleScalar(0);
        val = mxGetPr(plhs[0]);
        
		x1 = 1;
		y1 = 1;
		x2 = width;
		y2 = height;
        
        // compute the sum of the region
        x1 --;
        y1 --;
        x2 --;
        y2 --;
        if ((x1 > 0) && (y1 > 0)) {
            // general case
            *val =   integralImage[sub2ind(y2, x2, height)]
                   - integralImage[sub2ind(y1-1, x2, height)]
                   - integralImage[sub2ind(y2, x1-1, height)]
                   + integralImage[sub2ind(y1-1, x1-1, height)];
        } else if ((x1 > 0) && (y1 == 0)) {
            *val =  integralImage[sub2ind(y2, x2, height)]
                  - integralImage[sub2ind(y2, x1-1, height)];
        } else if ((x1 == 0) && (y1 > 0)) {
            *val =  integralImage[sub2ind(y2, x2, height)]
                  - integralImage[sub2ind(y1-1, x2, height)];
        } else {
            *val = integralImage[sub2ind(y2, x2, height)];
        }
	} else if (nrhs == 2) {
		// get the second argument <rect>
		rect = mxGetPr(prhs[1]);
        nRect = mxGetM(prhs[1]);
        
        // allocate output
        plhs[0] = mxCreateDoubleMatrix(1, nRect, mxREAL);
        val = mxGetPr(plhs[0]);
        
        for (iRect = 0; iRect < nRect; iRect++) {
            x1 = (int)rect[sub2ind(iRect, 0, nRect)];
            y1 = (int)rect[sub2ind(iRect, 1, nRect)];
            x2 = (int)rect[sub2ind(iRect, 2, nRect)];
            y2 = (int)rect[sub2ind(iRect, 3, nRect)];
            
            if ((x1 < 1) || (y1 < 1) || (x2 > width) || (y2 > height)) {
                mexErrMsgTxt("Required region is out of bound.");
            }
            
            // compute the sum of the region
            x1 --;
            y1 --;
            x2 --;
            y2 --;
            if ((x1 > 0) && (y1 > 0)) {
                // general case
                val[iRect] =   integralImage[sub2ind(y2, x2, height)]
                              - integralImage[sub2ind(y1-1, x2, height)]
                              - integralImage[sub2ind(y2, x1-1, height)]
                              + integralImage[sub2ind(y1-1, x1-1, height)];
            } else if ((x1 > 0) && (y1 == 0)) {
                val[iRect] =   integralImage[sub2ind(y2, x2, height)]
                              - integralImage[sub2ind(y2, x1-1, height)];
            } else if ((x1 == 0) && (y1 > 0)) {
                val[iRect] =  integralImage[sub2ind(y2, x2, height)]
                             - integralImage[sub2ind(y1-1, x2, height)];
            } else {
                val[iRect] = integralImage[sub2ind(y2, x2, height)];
            }
            
        }
	} else {
		mexErrMsgTxt("Invalid number of arguments");
	}
	
	

}
	                 
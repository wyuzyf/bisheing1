#include "mex.h"
#include "util.h"

void mexFunction(int nlhs, mxArray *plhs[],
		 int nrhs, const mxArray *prhs[])
{
  int rows, cols, i;
  double *p, *r;

  if(nrhs != 1)
    mexErrMsgTxt("Usage: x = randgamma(a)");

  /* prhs[0] is first argument.
   * mxGetPr returns double*  (data, col-major)
   * mxGetM returns int  (rows)
   * mxGetN returns int  (cols)
   */
  rows = mxGetM(prhs[0]);
  cols = mxGetN(prhs[0]);
  p = mxGetPr(prhs[0]);

  if(mxIsSparse(prhs[0]))
    mexErrMsgTxt("Cannot handle sparse matrices.  Sorry.");

  /* plhs[0] is first output */
  plhs[0] = mxCreateDoubleMatrix(rows, cols, mxREAL);
  r = mxGetPr(plhs[0]);
  for(i=0;i<rows*cols;i++) {
    *r++ = GammaRand(*p++);
  }
}



/*
This file contains general program definitions.

@author zhao lu
@version 20170413
*/

#ifndef DEFS_H
#define DEFS_H

/********************************* Includes **********************************/

/* From standard C library */
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <iostream>
//#include "unistd.h"

/* From OpenCV library */
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* From GSL */
//#include <gsl/gsl_rng.h>
//#include <gsl/gsl_randist.h>


/******************************* Defs and macros *****************************/

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef MIN
#define MIN(x,y) ( ( x < y )? x : y )
#endif

#ifndef MAX
#define MAX(x,y) ( ( x > y )? x : y )
#endif

#ifndef ABS
#define ABS(x) ( ( x < 0 )? -x : x )
#endif

/********************************** Structures *******************************/

#endif



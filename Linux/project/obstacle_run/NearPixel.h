#ifndef NEARPIXEL_H
#define NEARPIXEL_H

#include "opencv/cv.h"

#include <iostream>

using namespace cv;
using namespace std;

void lookNearByPixels(int x, int y,Scalar pix, Mat& img, int offset1, int offset2, int offset3);
bool singleInRange(Scalar pixel,Scalar lower,Scalar upper);
Scalar getRange(Scalar pixel,int offset1,int offset2,int offset3);
Mat applyDilatePixToMask(Mat &mask,Mat &img,int,int,int);

#endif
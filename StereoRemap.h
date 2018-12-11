//
// Created by liheng on 18-12-11.
//

#ifndef STEREOCLOUD_STEREOREMAP_H
#define STEREOCLOUD_STEREOREMAP_H


#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
/*
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "opencv2/calib3d//calib3d.hpp"
*/

//using namespace cv;

//#define MATLAB640X480
//#define  OPENCV1392X768
//#define
//define OPENCV1280X720
//#define MATLAB1280X720
//#define   OPENCV328
//#define   OPENCV331
//define   OPENCV40102
//#define   OPENCV40201
//#define   OPENCV409
//#define   OPENCV410
//#define   OPENCV411
//#define   OPENCV412
//#define   OPENCV41201
//#define MATLAB720X480
#define MATLABP

void calcParam(cv::Mat &frame);
void StereoRemap(cv::Mat &leftImage, cv::Mat &rightImage);
cv::Mat StereoGetQ();


#endif //STEREOCLOUD_STEREOREMAP_H

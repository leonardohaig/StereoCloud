//
// Created by liheng on 18-12-11.
//
#include "StereoRemap.h"
using namespace cv;
#ifdef MATLABP////////////////////////////////////////////////

double M1[9] = {
        739.997060224499,	0,	321.419545781160,
        0,	752.118946919696,	242.349967124807,
        0,	0,	1
};//KK_left

double M2[9] = {
        738.536012228155,	0,	327.721540983975,
        0,	749.398587880538,	216.060732919020,
        0,	0,	1
};//KK_right


double D1[5] = {-0.421354568744269,0.296623390986417,-0.00422501118607336,0.00171186543643695,0}; //kc_left
double D2[5] = {-0.368899579150550,0.0860300481370583,-0.00303009158837425,0.000595322056595916,0};  //kc_right

double arrayR[9] = {
        0.999995204849927,	0.000557795666914800,	-0.00304616827283225,
        -0.000511839756992768,	0.999886363416042,	0.0150664619173058,
        0.00305422612384878,	-0.0150648305213313,	0.999881854312872,
};      //R

double arrayT[3] = {-214.674194997180,0.967989972205085,-13.8288099569785}; //T


cv::Mat CM1(3, 3, CV_64FC1, M1);
cv::Mat CM2(3, 3, CV_64FC1, M2);
cv::Mat CD1(1, 5, CV_64FC1, D1);
cv::Mat CD2(1, 5, CV_64FC1, D2);
cv::Mat R(3, 3, CV_64FC1, arrayR);
cv::Mat T(3, 1, CV_64FC1, arrayT);

cv::Mat MX1 ;
cv::Mat MY1 ;
cv::Mat MX2 ;
cv::Mat MY2 ;
cv::Mat Q;

cv::Rect roi1, roi2;
#endif

void calcParam(cv::Mat &frame)
{
    int width = frame.cols/2;
    int height = frame.rows;
    cv::Size imageSize(width, height);
    cv::Size imageSize1(width, height);
    MX1 = cv::Mat(imageSize1, CV_32FC1);
    MY1 = cv::Mat(imageSize1, CV_32FC1);
    MX2 = cv::Mat(imageSize1, CV_32FC1);
    MY2 = cv::Mat(imageSize1, CV_32FC1);

    cv::Mat R1, R2, P1, P2;//, Q;

    cv::stereoRectify(
            CM1,
            CD1,
            CM2,
            CD2,
            imageSize,
            R,
            T,
            R1, R2, P1, P2, Q,
            CV_CALIB_ZERO_DISPARITY,
            1,
            imageSize1,
            &roi1, &roi2
    );

    if( CV_32FC1 != Q.type() )
        Q.convertTo(Q,CV_32FC1);

    Q.at<float>(3,2) = Q.at<float>(3,2) * 1000.0;

    initUndistortRectifyMap(
            CM1,
            CD1,
            R1, P1,
            imageSize1,
            CV_32FC1,
            MX1, MY1
    );
    initUndistortRectifyMap(
            CM2,
            CD2,
            R2, P2,
            imageSize1,
            CV_32FC1,
            MX2, MY2
    );
}

void StereoRemap(cv::Mat &leftImage, cv::Mat &rightImage)
{

    cv::remap(leftImage, leftImage, MX1, MY1, cv::INTER_LINEAR);
    cv::remap(rightImage, rightImage, MX2, MY2, cv::INTER_LINEAR);

//    cv::Rect roi;
//    roi.x = (roi1.x > roi2.x) ? roi1.x : roi2.x;
//    roi.y = (roi1.y > roi2.y) ? roi1.y : roi2.y;
//    roi.height = ((roi1.y+roi1.height) < (roi2.y+roi2.height)) ? (roi1.y+roi1.height-roi.y) : (roi2.y+roi2.height-roi.y);
//    roi.width = ((roi1.x+roi1.width) < (roi2.x+roi2.height)) ? (roi1.x+roi1.width-roi.x) : (roi2.x+roi2.width-roi.x);
//    cv::Mat leftROIImage(leftImage, roi);
//    cv::Mat rightROIImage(rightImage, roi);
//    leftImage = leftROIImage;
//    rightImage = rightROIImage;

}

cv::Mat StereoGetQ()
{
    return Q;
}

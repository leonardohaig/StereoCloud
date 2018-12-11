//
// Created by liheng on 18-12-11.
//

#ifndef STEREOCLOUD_STEREOVIDEOCAPTURE_H
#define STEREOCLOUD_STEREOVIDEOCAPTURE_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <mutex>


class StereoVideoCapture
{
public:
    StereoVideoCapture();
    virtual ~StereoVideoCapture();

private:
    cv::Mat m_OriginImage;//原始图像,CV_8UC3
    cv::Mat m_OriginLeftImage;
    cv::Mat m_OriginRightImage;

    cv::Mat m_leftImage;//左右图像，矫正后的，CV_8UC3
    cv::Mat m_rightImage;
    unsigned int m_curFrameIdx;//当前帧编号

    bool m_bSaveImage;//是否保存视频
    std::string m_strSavedVideoPath;//视频保存路径，包括后缀
    unsigned int m_nSavedFps;//保存视频的帧率


private:
    cv::VideoCapture m_videoCapture;
    cv::VideoWriter m_videoWriter;

private://与线程相关的变量
    std::thread m_captureThread;//视频捕捉线程
    std::mutex m_mutex;//互斥锁，防止获取的左右图像不是同一时刻的
    bool m_bEndCaptureThread;//是否结束视频捕捉线程


public:

    //打开设备，devIdx表示设备编号
    bool OpenCamera(int devIdx=0);

    //strVideoPath表示视频路径
    bool OpenCamera(std::string strVideoPath);

    //获取重投影矩阵，CV_32FC1类型，Note:其中涉及到距离的单位，已经改为 1/m.
    cv::Mat GetQ();

    //获得图像，此时图像已经矫正过
    bool CaptureStereoImage(cv::Mat& leftImage,cv::Mat& rightImage,unsigned int& curFrameIdx);

    //保存原始的视频，执行该函数后，程序将自动保存视频
    //返回值：true--可以保存视频，false--不可以保存视频
    bool StartSaveOriginImage(const std::string& strVideoPath,unsigned int nFps=15);

    //停止保存视频
    void StopSaveOriginImage();



    //=======================================================================================//
private:
    //以线程方式获得图像，此时图像已经矫正过
    void CaptureStereoImageInThread(cv::Mat& leftImage,cv::Mat& rightImage,unsigned int& curFrameIdx);

public://相机获取图像以线程形式运行

    //获得图像，此时图像已经矫正过
    bool GetStereoImage(cv::Mat& leftImage,cv::Mat& rightImage,unsigned int& curFrameIdx);

    //开启视频捕捉线程
    void StartCaptureThread();

    //结束视频捕捉线程//内联否？无必要
    void StopCaptureThread();

};


#endif //STEREOCLOUD_STEREOVIDEOCAPTURE_H

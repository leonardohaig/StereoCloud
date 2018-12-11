//
// Created by liheng on 18-12-11.
//

#include "StereoVideoCapture.h"
#include "StereoRemap.h"

StereoVideoCapture::StereoVideoCapture()
{
    m_curFrameIdx = 0;
    m_bEndCaptureThread = false;

    m_bSaveImage = false;
    m_strSavedVideoPath = "";
    m_nSavedFps = 15;
}


StereoVideoCapture::~StereoVideoCapture()
{
    m_bEndCaptureThread = true;

    m_bSaveImage = false;
    if( m_videoWriter.isOpened() )
        m_videoWriter.release();

    if( m_captureThread.joinable() )
        m_captureThread.join();

    if( m_videoCapture.isOpened() )
        m_videoCapture.release();


}

bool StereoVideoCapture::OpenCamera(int devIdx)
{
    m_videoCapture.open(devIdx);

    bool bCheck = m_videoCapture.isOpened();
    if( bCheck )
    {
        m_videoCapture>>m_OriginImage;
        calcParam(m_OriginImage);

    }
    else
    {
        std::cerr<<"Open the Stereo camera failed !"<<std::endl;
        exit(-1);
    }



    return bCheck;
}

//strVideoPath表示视频路径
bool StereoVideoCapture::OpenCamera(std::string strVideoPath)
{
    m_videoCapture.open(strVideoPath);

    bool bCheck = m_videoCapture.isOpened();
    if( bCheck )
    {
        m_videoCapture>>m_OriginImage;
        calcParam(m_OriginImage);

    }
    else
    {
        std::cerr<<"Open the Stereo camera failed !"<<std::endl;
        exit(-1);
    }



    return bCheck;
}

cv::Mat StereoVideoCapture::GetQ()
{
    return StereoGetQ();
}

bool StereoVideoCapture::CaptureStereoImage(cv::Mat& leftImage,cv::Mat& rightImage,unsigned int& curFrameIdx)
{
    std::unique_lock<std::mutex> lck(m_mutex);//加锁

    m_OriginImage.release();
    m_OriginLeftImage.release();
    m_OriginRightImage.release();

    m_leftImage.release();
    m_rightImage.release();


    m_videoCapture>>m_OriginImage;

    if( m_OriginImage.empty() )
        return false;

    m_OriginLeftImage = m_OriginImage(cv::Rect(0,0,672,376)).clone();
    m_OriginRightImage = m_OriginImage(cv::Rect(672,0,672,376)).clone();

    m_leftImage = m_OriginLeftImage.clone();
    m_rightImage = m_OriginRightImage.clone();

    StereoRemap(m_leftImage,m_rightImage);

    leftImage = m_leftImage.clone();
    rightImage = m_rightImage.clone();

    curFrameIdx = m_curFrameIdx;
    ++m_curFrameIdx;

    if( m_bSaveImage )
        m_videoWriter<<m_OriginImage;

    return true;
}

//保存原始的视频，执行该函数后，程序将自动保存视频
bool StereoVideoCapture::StartSaveOriginImage(const std::string& strVideoPath,unsigned int nFps)
{
    m_bSaveImage = true;
    m_strSavedVideoPath = strVideoPath;
    m_nSavedFps = nFps;

    m_videoWriter.open(strVideoPath, CV_FOURCC('M', 'J', 'P', 'G'), 25.0, m_OriginImage.size());
    return m_videoWriter.isOpened();
}

//停止保存视频
void StereoVideoCapture::StopSaveOriginImage()
{
    m_bSaveImage = false;
    if( m_videoWriter.isOpened() )
        m_videoWriter.release();
}

//以线程方式获得图像，此时图像已经矫正过
void StereoVideoCapture::CaptureStereoImageInThread(cv::Mat& leftImage,cv::Mat& rightImage,unsigned int& curFrameIdx)
{
    const int nFps = 25;//帧率
    while( !m_bEndCaptureThread )
    {
        CaptureStereoImage(leftImage,rightImage,curFrameIdx);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/nFps));
    }
}

bool StereoVideoCapture::GetStereoImage(cv::Mat& leftImage,cv::Mat& rightImage,unsigned int& curFrameIdx)
{
    std::unique_lock<std::mutex> lck(m_mutex);

    leftImage = m_leftImage.clone();
    rightImage = m_rightImage.clone();
    curFrameIdx = m_curFrameIdx;

    return ( !leftImage.empty() && !rightImage.empty());
}

//开启视频捕捉线程
void StereoVideoCapture::StartCaptureThread()
{
    unsigned int curFrameIdx;
    auto f = std::bind(&StereoVideoCapture::CaptureStereoImageInThread,this,
                       std::ref(m_leftImage),std::ref(m_rightImage),std::ref(m_curFrameIdx));

    m_captureThread = std::thread(f);
}

//结束视频捕捉线程
void StereoVideoCapture::StopCaptureThread()
{
    StopSaveOriginImage();
    m_bEndCaptureThread = true;
}

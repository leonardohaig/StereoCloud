//
// Created by liheng on 18-12-11.
//

#include <iostream>
#include "StereoCloudGenerator.h"
#include "StereoVideoCapture.h"
#include "def.h"



bool bEnd = false;
bool bNextPointCloud = true;//初值置为true,保证进入界面时有点云显示
//设置键盘交互函数,按下`space`键，某事发生
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
    //std::string str = event.getKeySym();
    //std::cout<<str<<std::endl;

    if( event.keyDown() )
    {
        if (event.getKeySym() == "Escape")
        {
            bEnd = true;
        }
        else if (event.getKeySym() == "space")
        {
            bNextPointCloud = true;
        }
    }


}

int ShowPointCloudFiles();

int main()
{

    //return ShowPointCloudFiles();

    StereoVideoCapture stereoVideoCapture;

#if B_USE_OFFLINE_VIDEO
    std::string strVideoPath;
    strVideoPath = "../StereoVideo.avi";
    if( !stereoVideoCapture.OpenCamera(strVideoPath) )
        return -1;
#else
    if( !stereoVideoCapture.OpenCamera(0) )
        return -1;
#endif



    StereoCloudGenerator kittiCloudGenerator;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbxyz(new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::Mat Q = stereoVideoCapture.GetQ();
    cv::Mat leftImag,rightImage;
    unsigned int frameIdx;
    stereoVideoCapture.StartCaptureThread();//开启视频捕捉线程


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
            (new pcl::visualization::PCLVisualizer("PointCloud")); //创建可视化窗口，名字叫做`Cloud Viewer: Rabbit`
    viewer->setBackgroundColor(0.0, 0, 0); //设置背景色为黑色
    viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
    viewer->initCameraParameters();   //初始化相机参数
    viewer->registerKeyboardCallback(keyboardEvent, nullptr);  //设置键盘回吊函数

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgbxyz, "sample cloud");


#if B_SAVE_VIDEO
    std::string strSavePath("../StereoVideo.avi");
    stereoVideoCapture.StartSaveOriginImage(strSavePath);
#endif

    while (!viewer->wasStopped() && !bEnd )
    {
        if( bNextPointCloud )
        {
            if( !stereoVideoCapture.GetStereoImage(leftImag,rightImage,frameIdx) )
                break;


            kittiCloudGenerator.cloudGenerator(leftImag, rightImage, Q, cloud_rgbxyz);

            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgbxyz, "sample cloud");

            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

            //bNextPointCloud = false;

#if B_SAVE_POINTCLOUD
            pcl::PCDWriter pcdWriter;
            std::string strPcdPath = "../PointCloudFiles/"+std::to_string(frameIdx) + ".pcd";
            pcdWriter.write<pcl::PointXYZRGB> (strPcdPath, *cloud_rgbxyz, false);

#endif

            //两张图片合并
            {
                //===绘制文本框====//
                //设置绘制文本的相关参数
                char text[256];
                sprintf(text, "Left Image");
                int font_face = cv::FONT_HERSHEY_COMPLEX;
                double font_scale = 1;
                int thickness = 2;
                int baseline;
                //获取文本框的长宽
                cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

                //将文本框居中绘制
                cv::Point origin;
                origin.x = 0;
                origin.y = 0 + text_size.height;
                cv::putText(leftImag, text, origin, font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);

                sprintf(text, "Right Image");
                cv::putText(rightImage, text, origin, font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
                cv::hconcat(leftImag, rightImage, leftImag);
            }

        }

        viewer->spinOnce(100);  //显示
        boost::this_thread::sleep(boost::posix_time::microseconds(100));   //随时间


        cv::imshow("Image after rectify", leftImag);
        cv::waitKey(1);


    }


    stereoVideoCapture.StopCaptureThread();//结束视频捕捉线程
    return  0;
}

int ShowPointCloudFiles()
{
    std::string pattern = "../PointCloudFiles/*.pcd";
    std::vector<cv::String> fn;
    glob(pattern, fn, true);

    std::vector<std::string> strFiles;
    strFiles.reserve( fn.size() );

    for(int i=0; i!=fn.size(); ++i )
        strFiles.push_back( fn[i] );

    std::sort(strFiles.begin(),strFiles.end());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbxyz(new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
            (new pcl::visualization::PCLVisualizer("PointCloud")); //创建可视化窗口，名字叫做`Cloud Viewer: Rabbit`
    viewer->setBackgroundColor(0.0, 0, 0); //设置背景色为黑色
    viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
    viewer->initCameraParameters();   //初始化相机参数
    viewer->registerKeyboardCallback(keyboardEvent, nullptr);  //设置键盘回吊函数



    if (-1 == pcl::io::loadPCDFile<pcl::PointXYZRGB>(strFiles[0], *cloud_rgbxyz))
    {
        return -1;
    }
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgbxyz, "sample cloud");

    int i=0;
    while (!viewer->wasStopped() && !bEnd )
    {

        if( bNextPointCloud )
        {
            if (-1 == pcl::io::loadPCDFile<pcl::PointXYZRGB>(strFiles[i++], *cloud_rgbxyz))
            {
                return -1;
            }

            bNextPointCloud = false;

            std::cout<<"the "<<i<<" point cloud file."<<std::endl;

            viewer->removeAllPointClouds();
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgbxyz, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        }


        viewer->spinOnce(100);  //显示
        boost::this_thread::sleep(boost::posix_time::microseconds(100));   //随时间

    }

    return 0;
}
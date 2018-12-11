//
// Created by liheng on 10/13/18.
//
#include <float.h>
#include "StereoCloudGenerator.h"
//#define FLT_EPSILON __FLT_EPSILON__

using namespace cv;

StereoCloudGenerator::StereoCloudGenerator(){
}

void StereoCloudGenerator::disparityMapGenerator(Mat leftImage, Mat rightImage,int minDisparity,
                                                int blockSize,int disp12MaxDiff,int preFilterCap, int uniquenessRatio,int speckleWindowSize,int speckleRange){
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>);
    int numDisparities=64-minDisparity;
    Mat leftImage_gray, rightImage_gray, disparity, true_disparity;
    int image_channels = leftImage_gray.channels();
    int P1 = 8*image_channels*blockSize*blockSize;
    int P2 = 32*image_channels*blockSize*blockSize;
    cv::cvtColor(leftImage, leftImage_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightImage, rightImage_gray, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
            minDisparity, numDisparities, blockSize,
            P1, P2, disp12MaxDiff, preFilterCap,
            uniquenessRatio, speckleWindowSize,
            speckleRange, cv::StereoSGBM::MODE_SGBM_3WAY
    );
    stereo->compute(leftImage_gray,rightImage_gray, disparity);

    disparity.convertTo(true_disparity, CV_32F, 1.0/16.0, 0.0);

    //imwrite("disparity.png",true_disparity);

    imshow("Disparity", true_disparity);
    waitKey();


}
void StereoCloudGenerator::cloudGenerator(Mat leftImage, Mat rightImage,Mat Q,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbxyz,
                                         int minDisparity,
                                         int blockSize,int disp12MaxDiff,int preFilterCap,
                                         int uniquenessRatio,int speckleWindowSize,int speckleRange)
{

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>);

    int numDisparities=64-minDisparity;
    Mat leftImage_gray, rightImage_gray, disparity, true_disparity;
    Mat cloud_xyz;

    uchar r, g, b;
    const float max_z = 50;

    int image_channels = leftImage_gray.channels();
    int P1 = 8*image_channels*blockSize*blockSize;
    int P2 = 32*image_channels*blockSize*blockSize;

    cv::cvtColor(leftImage, leftImage_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightImage, rightImage_gray, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
            minDisparity, numDisparities, blockSize,
            P1, P2, disp12MaxDiff, preFilterCap,
            uniquenessRatio, speckleWindowSize,
            speckleRange, cv::StereoSGBM::MODE_SGBM_3WAY
    );

    stereo->compute(leftImage_gray,rightImage_gray, disparity);

    disparity.convertTo(true_disparity, CV_32F, 1.0/16.0, 0.0);

    cv::reprojectImageTo3D(true_disparity, cloud_xyz, Q, true);
//    //=================================================//
//    float Q03 = Q.at<float>(0, 3);
//    float Q13 = Q.at<float>(1, 3);
//    float Q23 = Q.at<float>(2, 3);
//    float Q32 = Q.at<float>(3, 2);
//    float Q33 = Q.at<float>(3, 3);
//    for (int i = 0; i < disparity.rows; i++)
//    {
//        const float* disp_ptr = disparity.ptr<float>(i);
//        cv::Vec3f* out3D_ptr = cloud_xyz.ptr<cv::Vec3f>(i);
//
//        for (int j = 0; j < disparity.cols; j++)
//        {
//            const float pw = 1.0f / (disp_ptr[j] * Q32 + Q33);
//
//            cv::Vec3f& point = out3D_ptr[j];
//            point[0] = (static_cast<float>(j)+Q03) * pw;
//            point[1] = (static_cast<float>(i)+Q13) * pw;
//            point[2] = Q23 * pw;
//        }
//    }
//    //=================================================//




    depth_map.create(cloud_xyz.rows,cloud_xyz.cols,CV_32F);


    cloud_rgbxyz->clear();
    for(int i = 0; i < cloud_xyz.rows; i++)
    {
        for(int j = 0; j < cloud_xyz.cols; j++)
        {
            cv::Vec3f point = cloud_xyz.at<cv::Vec3f>(i, j);
            cv::Vec3b color = leftImage.at<cv::Vec3b>(i, j);

            //point /= 1000.0;

            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;

            pcl::PointXYZRGB point3D;

            point3D.x = point[0];
            point3D.y = point[1];
            point3D.z = point[2];

            depth_map.at<float>(i,j)=point[2];
            //cout<<point[1]<<" ";

            b = color[0];
            g = color[1];
            r = color[2];

            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 |
                            static_cast<uint32_t>(b));

            point3D.rgb = *reinterpret_cast<float*>(&rgb);

            cloud_rgbxyz->points.push_back (point3D);
        }
    }

#if 0
    {
        for(int i=0; i<100; ++i)
        {
            for(int j=0; j<100; ++j)
            {
                pcl::PointXYZRGB point3D;

                point3D.x = i;
                point3D.y = j;
                point3D.z = 0;

                r = 255;
                g = 0;
                b = 0;

                uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                                static_cast<uint32_t>(g) << 8 |
                                static_cast<uint32_t>(b));

                point3D.rgb = *reinterpret_cast<float*>(&rgb);

                cloud_rgbxyz->points.push_back (point3D);

            }

        }
    }
#endif


    //imwrite("depth.png",depth_map);
    cloud_rgbxyz->width=cloud_rgbxyz->points.size();
    cloud_rgbxyz->height=1;
    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZRGB> ("out_file.pcd", *cloud_rgbxyz, false);

    //cv::imshow("dispartyp",disparity);
    //cv::waitKey(0);
}
Mat StereoCloudGenerator::getDepthMap(){
    return depth_map;
}
StereoCloudGenerator::~StereoCloudGenerator(){}
# 说明
1.代码名称：*StereoCloud*
2.代码作用：*实时读取双目相机的图像，采用立体匹配算法进行三维重建，构建三维空间点云进行显示*
3.代码依赖：*OpenCV3.0或者以上版本（不依赖contrib模块），PCL。该代码可跨平台。*
4.程序运行流程说明：
4.1 视频捕捉。*开启双目设备，并开始视频捕捉线程，该线程一直运行，直至程序退出时结束。视频捕捉线程完成左右图像的获取以及图像畸变纠正，保证输出的左右图像共面且行对准。输入图像格式为：CV_8UC3类型。*
4.2 立体匹配及点云构建。*程序采用StereoSGBM立体匹配算法对左右图像进行匹配求视差，然后根据重投影矩阵和视差图得到各点的三维坐标及深度图；最后，利用各点的三维坐标及其在左图像的RGB颜色值构建三维点云，点云类型为PointXYZRGB格式。*
4.3 点云刷新显示。*对上一步求得的点云采用PCLVisualizer进行显示。*
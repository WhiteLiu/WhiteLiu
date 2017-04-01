#include "common.h"


int main()
{
    Mat rgb1,rgb2,depth1,depth2;
    fileread(rgb1,rgb2,depth1,depth2);

    CAMERA_INTRINSIC_PARAMETERS C;
    C.cx = 325.5;
    C.cy = 253.5;
    C.fx = 518.0;
    C.fy = 519.0;
    C.scale = 1000.0;

    //feature detector and descriptor compute
    Eigen::Isometry3d T = transformEstimation(rgb1,rgb2,depth1,depth2,C);

    PointCloud::Ptr cloud1 = image2PointCloud(rgb1,depth1,C);
    PointCloud::Ptr cloud2 = image2PointCloud(rgb2,depth2,C);

    //pcl::io::savePCDFile("1.pcd", *cloud1);
    //pcl::io::savePCDFile("2.pcd", *cloud2);

    cout<<"combining clouds"<<endl;
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *cloud2, *output, T.matrix());
    *cloud1 += *output;
    pcl::io::savePCDFile("result.pcd", *cloud1);
    cout<<"Final result saved."<<endl;
    return 0;
}

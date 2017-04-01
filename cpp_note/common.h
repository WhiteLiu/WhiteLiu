#ifndef COMMON_H
#define COMMON_H


#include <iostream>
#include <math.h>
#include <algorithm>

//OpenCV
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//Eigen
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;


// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>


//G2O
#include "g2o/core/block_solver.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"


#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>



using namespace g2o;


struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
};


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void fileread(Mat& rgb1,Mat& rgb2,Mat& depth1,Mat& depth2);

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

Point3f point2dTo3d( Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

void extractKeypointsAndDescripotrs(const Mat& rgb1,const Mat& rgb2,vector<KeyPoint>& keyPts1,vector<KeyPoint>& keyPts2,Mat& descriptors1,Mat& descriptors2,const string featureString = "ORB",const string descriptorString ="BRIEF");

void descriptorsMatch(const Mat& rgb1,const Mat& rgb2,const vector<KeyPoint>& keyPts1,const vector<KeyPoint> keyPts2,const Mat& descriptors1,const Mat& descriptors2,vector<DMatch>& matches);

void getObjectPointsAndImagePoints(const Mat& depth1,const Mat& depth2,const vector<KeyPoint>& keyPts1,const vector<KeyPoint>& keyPts2,const vector<DMatch>& matches,vector<Point2f>& imagePoints,vector<Point3f>& objectPoints,vector<Eigen::Vector2d>& imagePoints1,vector<Eigen::Vector2d>& imagePoints2,const CAMERA_INTRINSIC_PARAMETERS& C);

Eigen::Isometry3d transformEstimation(const Mat& rgb1,const Mat& rgb2,const Mat& depth1,const Mat& depth2,const CAMERA_INTRINSIC_PARAMETERS& C);

void BundleAdjustmentOptimization(const vector<Point3f>& objectPoints,const vector<Eigen::Vector2d>& imagePoints1,const vector<Eigen::Vector2d>& imagePoints2);

#endif

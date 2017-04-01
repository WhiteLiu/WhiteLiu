#include "common.h"

void fileread(Mat& rgb1,Mat& rgb2,Mat& depth1,Mat& depth2)
{
//    rgb1 = imread("/home/jinhuazhe/Downloads/liu/rgbd_dataset_freiburg1_xyz/rgb/1305031102.175304.png");
//    rgb2 = imread("/home/jinhuazhe/Downloads/liu/rgbd_dataset_freiburg1_xyz/rgb/1305031102.211214.png");
//    depth1 = imread("/home/jinhuazhe/Downloads/liu/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png");
//    depth2 = imread("/home/jinhuazhe/Downloads/liu/rgbd_dataset_freiburg1_xyz/depth/1305031102.226738.png");




    rgb1 = imread("/home/jinhuazhe/Downloads/liu/slambook/ch5/joinMap/color/3.png");
    depth1 = imread("/home/jinhuazhe/Downloads/liu/slambook/ch5/joinMap/depth/3.pgm");
    rgb2 = imread("/home/jinhuazhe/Downloads/liu/slambook/ch5/joinMap/color/4.png");
    depth2 = imread("/home/jinhuazhe/Downloads/liu/slambook/ch5/joinMap/depth/4.pgm");
}


PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m+=2)
        for (int n=0; n < depth.cols; n+=2)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

Point3f point2dTo3d( Point3f& point,const  CAMERA_INTRINSIC_PARAMETERS& camera )
{
    Point3f p;
    p.z = float( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}


void extractKeypointsAndDescripotrs(const Mat& rgb1,const Mat& rgb2,vector<KeyPoint>& keyPts1,vector<KeyPoint>& keyPts2,Mat& descriptors1,Mat& descriptors2,const string featureString,const string descriptorString )
{
    Ptr<FeatureDetector> detector = FeatureDetector::create(featureString);
    Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptorString);

    detector->detect(rgb1,keyPts1);
    detector->detect(rgb2,keyPts2);
    descriptor->compute(rgb1,keyPts1,descriptors1);
    descriptor->compute(rgb2,keyPts2,descriptors2);
}

void descriptorsMatch(const Mat& rgb1,const Mat& rgb2,const vector<KeyPoint>& keyPts1,const vector<KeyPoint> keyPts2,const Mat& descriptors1,const Mat& descriptors2,vector<DMatch>& matches)
{
    //descriptor match
    vector<DMatch> des_match;
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    matcher->match(descriptors1,descriptors2,des_match);
    Mat out;

    drawMatches(rgb1,keyPts1,rgb2,keyPts2,des_match,out);
    imshow("matches",out);
    waitKey();

    float dis_min = 1000,dis_max;
    for(size_t i = 0; i<des_match.size();i++)
    {
        float dis = des_match[i].distance;
        if(dis_min>dis) dis_min = dis;
        if(dis_max<dis) dis_max = dis;
    }


    for(size_t i = 0;i<des_match.size();i++)
    {
        float dis = des_match[i].distance;
        if(dis<4*dis_min) matches.push_back(des_match[i]);
    }
    drawMatches(rgb1,keyPts1,rgb2,keyPts2,matches,out,Scalar(0,255,0));
    imshow("matches",out);
    waitKey();

}

void  getObjectPointsAndImagePoints(const Mat& depth1,const Mat& depth2,const vector<KeyPoint>& keyPts1,const vector<KeyPoint>& keyPts2,const vector<DMatch>& matches,vector<Point2f>& imagePoints,vector<Point3f>& objectPoints,vector<Eigen::Vector2d>& imagePoints1,vector<Eigen::Vector2d>& imagePoints2,const CAMERA_INTRINSIC_PARAMETERS& C)
{
    for(size_t i=0;i<matches.size();i++)
    {
        int m = matches[i].queryIdx;
        int n = matches[i].trainIdx;
        int x1 = keyPts1[m].pt.x;
        int y1 = keyPts1[m].pt.y;
        Point2f p = keyPts1[m].pt;
        int x = keyPts2[n].pt.x;
        int y = keyPts2[n].pt.y;

        ushort d = ((ushort*) depth1.data)[y*depth1.cols+x];
        if(d == 0) continue;
        Eigen::Vector2d p1;
        p1<<x1,y1;
        Eigen::Vector2d p2;
        p2<<x,y;
        imagePoints1.push_back(p1);
        imagePoints2.push_back(p2);

        Point3f q(x1,y1,d);

        imagePoints.push_back(p);
        Point3f q1 = point2dTo3d(q,C);
        objectPoints.push_back(q1);
        //cout<<"abcd"<<q1<<endl;

    }
}


Eigen::Isometry3d transformEstimation(const Mat& rgb1,const Mat& rgb2,const Mat& depth1,const Mat& depth2,const CAMERA_INTRINSIC_PARAMETERS& C)
{
    vector<KeyPoint> keyPts1,keyPts2;
    Mat descriptors1,descriptors2;

    extractKeypointsAndDescripotrs(rgb1,rgb2,keyPts1,keyPts2,descriptors1,descriptors2);


    vector<DMatch> matches;
    descriptorsMatch(rgb1,rgb2,keyPts1,keyPts2,descriptors1,descriptors2,matches);


    vector<Point2f> points;
    vector<Point3f> objectPoints;
    vector<Eigen::Vector2d> imagePoints1,imagePoints2;

    getObjectPointsAndImagePoints(depth1,depth2,keyPts1,keyPts2,matches,points,objectPoints,imagePoints1,imagePoints2,C);

    Mat translation,rotation;
    double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}    };

    Mat cameraMatrix(3,3,CV_64F,camera_matrix_data);


    solvePnPRansac(objectPoints,points,cameraMatrix,Mat(),rotation,translation,false, 100, 1.0, 20);


    Mat rot;
    Rodrigues(rotation,rot);

    Eigen::Matrix3d r;
    Eigen::Vector3d t;

    cout<<rot<<endl;
    cout<<translation<<endl;

    r<< ((double*)rot.data)[0],((double*)rot.data)[1],((double*)rot.data)[2],
            ((double*)rot.data)[3],((double*)rot.data)[4],((double*)rot.data)[5],
            ((double*)rot.data)[6],((double*)rot.data)[7],((double*)rot.data)[8];
    t<<((double*)translation.data)[0],((double*)translation.data)[1],((double*)translation.data)[2];

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(r);
    T.pretranslate(t);
    cout<<T.matrix()<<endl;

    BundleAdjustmentOptimization(objectPoints,imagePoints1,imagePoints2);

    return T;
}



void BundleAdjustmentOptimization(const vector<Point3f>& objectPoints,const vector<Eigen::Vector2d>& imagePoints1,const vector<Eigen::Vector2d>& imagePoints2)
{

    typedef BlockSolver_6_3 SlamBlockSolver;
    typedef LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

//    Eigen::Quaternionf r = Eigen::Quaterniond(T.rotation());
//    Eigen::Quaterniond rr(r.w(),r.x(),r.y(),r.z());
//    Eigen::Vector3f t = T.translation();
//    Eigen::Vector3d tt(t[0],t[1],t[2]);

//    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
//    //SlamLinearSolver* linearSolver = new SlamLinearSolver();
//    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
//    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

//    //OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(blockSolver);

//    //Eigen::Vector2d principal_point(325.5,253.5);
//    Eigen::Vector2d principal_point(320,240);
//    //CameraParameters * cam_params = new CameraParameters (518.5, principal_point,0);
//    CameraParameters * cam_params = new CameraParameters (518.5, principal_point,0);
//    cam_params->setId(0);
//    SparseOptimizer optimizer;
//    optimizer.setAlgorithm(solver);
//    optimizer.setVerbose(false);
//    optimizer.addParameter(cam_params);



//    int id=0;

//    for(int i =0;i<2;i++)
//    {

//        VertexSE3Expmap* se3 = new VertexSE3Expmap();
//        se3->setId(id);
//        if(i == 0)
//        {
//            se3->setFixed(true);
//            se3->setEstimate(SE3Quat());
//        }
//        else
//        {  //SE3Quat* pos = new SE3Quat(rr,tt);
//            se3->setEstimate(SE3Quat());
//        }
//        optimizer.addVertex(se3);
//        id++;
//    }

//    cout<<"obejecPoints.size = ";
//    cout<<objectPoints.size()<<endl;
//    for(int i =0;i<objectPoints.size();i++)
//    {
//        VertexSBAPointXYZ* v_xyz = new VertexSBAPointXYZ();
//        Eigen::Vector3d vec;
//        v_xyz->setId(id);
//        vec<<objectPoints[i].x,objectPoints[i].y,objectPoints[i].z;
//        v_xyz->setEstimate(vec);
//        v_xyz->setMarginalized(false);
//        optimizer.addVertex(v_xyz);
//        id++;

//    }


//    cout<<imagePoints1.size()<<endl;
//    for(int i =0;i<imagePoints1.size();i++)
//    {
//        EdgeProjectXYZ2UV* e = new EdgeProjectXYZ2UV();
//        e->setVertex(0, dynamic_cast<VertexSBAPointXYZ*> (optimizer.vertex(2+i)));
//        e->setVertex(1, dynamic_cast<VertexSE3Expmap*> (optimizer.vertex(0)));
//        e->setMeasurement(imagePoints1[i]);
//        e->information() = Eigen::Matrix2d::Identity();
//        e->setParameterId(0,0);
//        RobustKernelHuber* rk = new RobustKernelHuber;
//        e->setRobustKernel(rk);
//        optimizer.addEdge(e);
//    }


//    cout<<imagePoints2.size()<<endl;
//    for(int i =0;i<imagePoints2.size();i++)
//    {
//        EdgeProjectXYZ2UV* e = new EdgeProjectXYZ2UV();
//        e->setVertex(0, dynamic_cast<VertexSBAPointXYZ*> (optimizer.vertex(2+i)));
//        e->setVertex(1, dynamic_cast<VertexSE3Expmap*> (optimizer.vertex(1)));
//        e->setMeasurement(imagePoints2[i]);
//        e->information() = Eigen::Matrix2d::Identity();
//        RobustKernelHuber* rk = new RobustKernelHuber;
//        e->setParameterId(0,0);
//        e->setRobustKernel(rk);
//        optimizer.addEdge(e);
//    }

    g2o::SparseOptimizer    optimizer;
    // 使用Cholmod中的线性方程求解器
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
    // 6*3 的参数
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
    // L-M 下降
    //g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );
    OptimizationAlgorithmDogleg* algorithm = new OptimizationAlgorithmDogleg(block_solver);
    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( false );


    g2o::CameraParameters* camera = new g2o::CameraParameters( 518.5, Eigen::Vector2d(325,253), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );
    // 添加节点
    // 两个位姿节点
    for ( int i=0; i<2; i++ )
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); // 第一个点固定为零
        // 预设值为单位Pose，因为我们不知道任何信息
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
    }
    // 很多个特征点的节点
    // 以第一帧为准
    for ( size_t i=0; i<objectPoints.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );
        // 由于深度不知道，只能把深度设置为1了

        Eigen::Vector3d vec;
        vec<<objectPoints[i].x,objectPoints[i].y,objectPoints[i].z;
        v->setMarginalized(true);
        v->setEstimate( vec );
        optimizer.addVertex( v );
    }


    // 准备边
    // 第一帧
    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<imagePoints1.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
        edge->setMeasurement( imagePoints1[i] );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    // 第二帧
    for ( size_t i=0; i<imagePoints2.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
        edge->setMeasurement( imagePoints2[i] );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }

    int inliers=0;
    for ( vector<g2o::EdgeProjectXYZ2UV*>::iterator e = edges.begin();e!=edges.end();e++ )
    {
        (*e)->computeError();
        // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
        if ( (*e)->chi2() > 1 )
        {
            cout<<"error = "<<(*e)->chi2()<<endl;
        }
        else
        {
            inliers++;
        }
    }


    optimizer.setVerbose(true);
    optimizer.initializeOptimization();

    cout<<"optimizer start"<<endl;
    optimizer.optimize(1,true);
    cout<<"optimizer end"<<endl;

    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    cout<<"Pose="<<endl<<pose.matrix()<<endl;


    for ( size_t i=0; i<imagePoints1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
        cout<<"vertex id "<<i+2<<", pos = ";
        Eigen::Vector3d pos = v->estimate();
        cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    }
}







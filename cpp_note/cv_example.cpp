#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
int main()
{

    Mat a;
    a.create(100,100,CV_8UC1);
    a.setTo(255);
    imshow("a",a);
    cvWaitKey(0);

    Mat b(a,Rect(20,20,10,10));
    b.setTo(0);
    imshow("a",a);
    cvWaitKey(0);
    Mat c = a(Range(3,60),Range(20,80));
    c.setTo(0);

    imshow("a",a);
    cvWaitKey(0);

    Mat r = Mat(800,1280,CV_8UC3);
    randu(r,Scalar::all(0),Scalar::all(255));
    imshow("a",r);
    imwrite("r.jpg",r);
    cvWaitKey(0);

    return 0;
}

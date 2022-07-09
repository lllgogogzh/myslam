#include "Convert.h"


Convert::Convert()
{
        //Camera.fx: 517.306408
        //Camera.fy: 516.469215
        //Camera.cx: 318.643040
        //Camera.cy: 255.313989
        mK<<517.306408,0.0,318.643040,
        0.0,516.469215,255.313989,
        0.0,0.0,1.0;
}

void Convert::Camera2Pixel(const Eigen::Vector3d pts,Eigen::Vector2d &pixel)
{
    Eigen::Vector3d tpixel = mK*pts;
    double tdepth = tpixel(2);
    if(tdepth!=0)
    {
        tpixel(0)=tpixel(0)/tdepth;
        tpixel(1)=tpixel(1)/tdepth;
        tpixel(2)=1.0;
    }
    else
    {
        ROS_ERROR("error!   tdepth == 0 !!!");
        return;
    }
    pixel=Eigen::Vector2d(tpixel(0),tpixel(1));
}

void Convert::Pixel2Camera(const Eigen::Vector2d pixel,Eigen::Vector3d &pts,double d)
{
    double x=(pixel(0)-mK(0,2))/mK(0,0);
    double y=(pixel(1)-mK(1,2))/mK(1,1);

    pts=Eigen::Vector3d(x*d,y*d,d);
}

void Convert::Camera2World(const Eigen::Vector3d camera,Eigen::Vector3d &world,const cv::Mat Tcw)
{
    Eigen::Matrix4d eTcw;
    eTcw<<Tcw.at<double>(0,0),Tcw.at<double>(0,1),Tcw.at<double>(0,2),Tcw.at<double>(0,3),
                    Tcw.at<double>(1,0),Tcw.at<double>(1,1),Tcw.at<double>(1,2),Tcw.at<double>(1,3),
                    Tcw.at<double>(2,0),Tcw.at<double>(2,1),Tcw.at<double>(2,2),Tcw.at<double>(2,3),
                    Tcw.at<double>(3,0),Tcw.at<double>(3,1),Tcw.at<double>(3,2),Tcw.at<double>(3,3);
    Eigen::Vector4d qiciWorld,qiciCamera;
    qiciCamera<<camera(0),camera(1),camera(2),1.0;
    qiciWorld = eTcw.inverse()*qiciCamera;

    world = qiciWorld.block(0,0,3,1);
}






#ifndef VISUALODOM_H
#define VISUALODOM_H
#include <iostream>
#include <memory>


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include<nav_msgs/Odometry.h>

#include <Optimizer.h>



#include "Frame.h"
#include "debug.h"
namespace GzhSLAM
{
    

class VisualOdom
{
    public:
       VisualOdom();
        ~VisualOdom(){;};
        VisualOdom(ros::NodeHandle &n);

        void TrackRGBD(cv::Mat &colorImg,cv::Mat &depthImg,double t);

        void FeatureMatch();
        void TrackByPnP();
        void RecoverDepth();
        void PubPoseOnce();
        void Optimization();
        cv::Ptr<cv::DescriptorMatcher> mDescriptorMatcher;
        std::vector<cv::DMatch> mvGoodDMatchPoints;
        std::vector<cv::DMatch> mvDMatchPoints;


        std::vector<cv::Point3d> mvPointsInWorld;
        std::vector<cv::Point2d> mvKeyPointsInCurFrame;//use for matches and pnp


        cv::Mat mTcw;

       static const int RGBD = 1;

    public:
        //for status check
        enum Status
        {
            INITIALIZE = 0,
            NORMAL = 1,
            ERROR = -1
        };
        int mnVOStatus;

        //Frame *mpCurrentFrame;
        //Frame *mpPrevFrame;
        std::shared_ptr<Frame> mpCurrentFrame;
        std::shared_ptr<Frame> mpPrevFrame;

        std::shared_ptr<Convert> mpConverter;


    public:
        //for ros public
        tf::TransformBroadcaster br;
        ros::NodeHandle nh;
        ros::Publisher posePub;
        nav_msgs::Odometry pubOdom;
};



}





#endif
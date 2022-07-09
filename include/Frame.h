#ifndef FRAME_H
#define FRAME_H

#include <iostream>
#include <ros/ros.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "debug.h"
#include "Convert.h"

namespace GzhSLAM
{



class Frame
{
    public:
        Frame(){;};
        ~Frame(){;};

        Frame(cv::Mat &color,cv::Mat &depth,bool isExtractORB,double mdTime);

    public:
        cv::Mat mColorImgMat;
        cv::Mat mDepthImgMat;
        double mdTime;


        std::vector<cv::KeyPoint> mvKeyPoints;
        cv::Mat mDescriptions;
        cv::Ptr<cv::FeatureDetector> mFASTDector;
        cv::Ptr<cv::DescriptorExtractor> mDescriptionExtractor;
        cv::Ptr<cv::DescriptorMatcher> mDescriptorMatcher;

        std::vector<cv::Point3d> mvKeyPoints3D;


        int EightDirection[8][2]={{1,1},{1,0},{1,-1},{0,1},{0,-1},{-1,0},{-1,-1},{-1,1}};

    public:
        void getKeyPointsDepth();
        bool isOverWide(double x,double y,const cv::Mat depth);
        void recoverCameraPoints3D(const double x,const double y,double d);


    public:
        cv::Mat mTcw;

};




}


#endif
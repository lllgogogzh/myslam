#ifndef CONVERT_H
#define CONVERT_H

#include <ros/ros.h>


#include "opencv2/core/core.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"

class Convert
{
    public:
        Convert();
        ~Convert(){;};


        void Camera2Pixel(const Eigen::Vector3d pts,Eigen::Vector2d &pixel);
        void Pixel2Camera(const Eigen::Vector2d pixel,Eigen::Vector3d &pts,double d);
        void World2Camera();
        void Camera2World(const Eigen::Vector3d camera,Eigen::Vector3d &world,const cv::Mat Tcw);

        Eigen::Matrix3d mK;
};





#endif
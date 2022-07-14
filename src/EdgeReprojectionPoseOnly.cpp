#include "EdgeReprojectionPoseOnly.h"
#include "VertexPose.h"
#include <Eigen/Core>
void EdgeReprojectionPoseOnly::ComputeJacobians()
{
    //Sophus se3   :  first trans   , then rotate
    Eigen::Matrix<double,2,6> jacobian;

    Vec3 ptsw = landmark_world_;

    auto vertex = verticies_[0];
    Vec6 param = vertex->Parameters();

    Sophus::SE3 Tcw = Sophus::SE3::exp(param);
    Eigen::Matrix3d Rcw = Tcw.matrix().block(0,0,3,3);
    Eigen::Vector3d tcw = Tcw.matrix().block(0,3,3,1);

    Vec3 ptsc = Rcw * ptsw + tcw;
    
    double xc ,yc ,zc ,zc2;
    xc = ptsc(0); 
    yc = ptsc(1); 
    zc = ptsc(2) ;
    zc2 = zc*zc;

    double fx,fy;
    fx = K_(0,0);
    fy = K_(1,1);

    jacobian(0,0)=fx/zc;
    jacobian(0,1)=0.0;
    jacobian(0,2)=-fx*xc/zc2;
    jacobian(0,3)=-fx*xc/zc2*yc;
    jacobian(0,4)=fx+fx*xc*xc/zc2;
    jacobian(0,5)=-fx*yc/zc;

    jacobian(1,0)=0.0;
    jacobian(1,1)=fy/zc;
    jacobian(1,2)=-fy*yc/zc2;
    jacobian(1,3)=-fy-fy*yc*yc/zc2;
    jacobian(1,4)=fy*xc*yc/zc2;
    jacobian(1,5)=fy*xc/zc;

    jacobians_[0] = -1*jacobian;
}


void EdgeReprojectionPoseOnly::ComputeResidual()
{
    VecX pose_params = verticies_[0]->Parameters();
    Sophus::SE3 pose = Sophus::SE3::exp(pose_params);

    Vec3 pc = pose.matrix().block(0,0,3,3) * landmark_world_ + pose.matrix().block(0,3,3,1);
    pc = pc / pc[2];
    Vec2 pixel = (K_ * pc).head<2>() - observation_;
    // TODO:: residual_ = ????
    residual_ = pixel;
}


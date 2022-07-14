#ifndef EDGEREPROJECTIONXYZ_H
#define EDGEREPROJEXTIONXYZ_H


#include "Edge.h"

using namespace GzhSLAM;

/**
* 此边是视觉重投影误差，此边为二元边，与之相连的顶点有：
* 路标点的世界坐标系XYZ、观测到该路标点的 Camera 的位姿T_World_From_Body1
* 注意：verticies_顶点顺序必须为 XYZ、T_World_From_Body1。
*/
class EdgeReprojectionXYZ : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeReprojectionXYZ(const Vec3 &pts_i)
        : Edge(2, 2, std::vector<std::string>{"VertexXYZ", "VertexPose"}) {
        obs_ = pts_i;
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override { return "EdgeReprojectionXYZ"; }

    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

    void SetTranslationImuFromCamera(Eigen::Quaterniond &qic_, Vec3 &tic_);

private:
    //Translation imu from camera
    Qd qic;
    Vec3 tic;

    //measurements
    Vec3 obs_;
};







#endif
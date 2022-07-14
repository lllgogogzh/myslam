#ifndef EDGEREPROJECTIONPOSEONLY_H
#define EDGEREPROJECTIONPOSEONLY_H

#include "Edge.h"

using namespace GzhSLAM;


/**
 * 仅计算重投影pose的例子
 */
class EdgeReprojectionPoseOnly : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeReprojectionPoseOnly(const Vec3 &landmark_world, const Mat33 &K) :
        Edge(2, 1, std::vector<std::string>{"VertexPose"}),
        landmark_world_(landmark_world), K_(K) {}

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override { return "EdgeReprojectionPoseOnly"; }

    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

private:
    Vec3 landmark_world_;
    Mat33 K_;
};



#endif
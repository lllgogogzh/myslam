#ifndef GZHSLAM_VERTEXPOSE_H
#define GZHSLAM_VERTEXPOSE_H


#include "Vertex.h"

using namespace GzhSLAM;


class VertexPose : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPose() : Vertex(6, 6) {}

    /// 加法，可重定义
    /// 默认是向量加
    virtual void Plus(const VecX &delta) override;

    std::string TypeInfo() const {
        return "VertexPose";
    }

    /**
     * 需要维护[H|b]矩阵中的如下数据块
     * p: pose, m:mappoint
     * 
     *     Hp1_p2    
     *     Hp2_p2    Hp2_m1    Hp2_m2    Hp2_m3     |    bp2
     *                         
     *                         Hm2_m2               |    bm2
     *                                   Hm2_m3     |    bm3
     * 1. 若该Camera为source camera，则维护vHessionSourceCamera；
     * 2. 若该Camera为measurement camera, 则维护vHessionMeasurementCamera；
     * 3. 并一直维护m_HessionDiagonal；
     */
};




    #endif
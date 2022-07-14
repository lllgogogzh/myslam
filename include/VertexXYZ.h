#ifndef GZLSLAM_VERTEXXYZ_H
#define GZHSLAM_VERTEXXYZ_H


#include "Vertex.h"

using namespace GzhSLAM;

/**
 * @brief 以xyz形式参数化的顶点
 */
class VertexPointXYZ : public Vertex 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPointXYZ() : Vertex(3) {}

    std::string TypeInfo() const { return "VertexXYZ"; }
};



#endif
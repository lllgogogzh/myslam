#include "Vertex.h"
#include "Edge.h"
//#include <glog/logging.h>
#include <iostream>

using namespace std;

namespace GzhSLAM {

unsigned long global_edge_id = 0;

Edge::Edge(int residual_dimension, int num_verticies,
           const std::vector<std::string> &verticies_types) 
{
    residual_.resize(residual_dimension, 1);

//    verticies_.resize(num_verticies);      // TODO:: 这里可能会存在问题，比如这里resize了3个空,后续调用edge->addVertex. 使得vertex前面会存在空元素
    if (!verticies_types.empty())
    {
        verticies_types_ = verticies_types;
    }
    

    jacobians_.resize(num_verticies);
    id_ = global_edge_id++;

    Eigen::MatrixXd information(residual_dimension, residual_dimension);
    information.setIdentity();
    information_ = information;

}

Edge::~Edge() {}

double Edge::Chi2() 
{
    // TODO::  we should not Multiply information here, because we have computed Jacobian = sqrt_info * Jacobian
    return residual_.transpose() * information_ * residual_;
//    return residual_.transpose() * residual_;   // 当计算 residual 的时候已经乘以了 sqrt_info, 这里不要再乘
}



bool Edge::CheckValid() 
{
    if (!verticies_types_.empty()) 
    {
        // check type info
        for (size_t i = 0; i < verticies_.size(); ++i)
        {
            if (verticies_types_[i] != verticies_[i]->TypeInfo()) 
            {
                cout << "Vertex type does not match, should be " << verticies_types_[i] <<
                     ", but set to " << verticies_[i]->TypeInfo() << endl;
                return false;
            }
        }
    }
    
    return true;
}


}
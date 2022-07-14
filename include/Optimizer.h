#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include<vector>


#include <ProblemSolver.h>


class Optimizer
{
    public:
        Optimizer();
        ~Optimizer(){;};

        //TODO : Derive BA optimization by hand
        
        enum OptimizeType
        {
            ONLYPOSE = 0,
            POSEANDFEATURE =1
        };

        bool OptimizePoseOnly(int type , Eigen::Matrix4d Tcw ,  
                                                        std::vector<Eigen::Vector2d> kpts , std::vector<Eigen::Vector3d> ptsw,const Eigen::Matrix3d K,Eigen::Matrix4d &TTT);
};

//TODO : class edge just like g2o
 
//TODO : class vertex just like g2o

class VertexBase
{
    public:

};





#endif
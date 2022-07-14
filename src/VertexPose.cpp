#include "VertexPose.h"

void VertexPose::Plus(const VecX &delta) 
{
    /*VecX &parameters = Parameters();
    parameters.head<3>() += delta.head<3>();
    Qd q(parameters[6], parameters[3], parameters[4], parameters[5]);
    q = q * Sophus::SO3::exp(Vec3(delta[3], delta[4], delta[5])).unit_quaternion();  // right multiplication with so3
    q.normalized();
    parameters[3] = q.x();
    parameters[4] = q.y();
    parameters[5] = q.z();
    parameters[6] = q.w();*/
    VecX &parameters = Parameters();
    Sophus::SE3 prevT = Sophus::SE3::exp(parameters);
    Sophus::SE3 deltaT = Sophus::SE3::exp(delta);
    Sophus::SE3 updateT = deltaT*prevT;
    Vec6 update_se3 = updateT.log();
    for(int i=0;i<6;++i)
    {
        parameters_[i] = update_se3(i);
    }
}
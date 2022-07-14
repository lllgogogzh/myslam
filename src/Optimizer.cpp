#include "Optimizer.h"

#include <iostream>

Optimizer::Optimizer()
{
    ;
}


bool Optimizer::OptimizePoseOnly(int type , Eigen::Matrix4d Tcw ,  std::vector<Eigen::Vector2d> kpts , 
                                                                        std::vector<Eigen::Vector3d> ptsw,const Eigen::Matrix3d K,Eigen::Matrix4d &TTT)
{
    //ProblemSolver solver;


    std::shared_ptr<ProblemSolver> psolver = std::make_shared<ProblemSolver>();

    int edgeIndex = 0;
    if(type == Optimizer::ONLYPOSE)
    {
        //EdgeReprojectionPoseOnly e(ptsw[0],K);
        unsigned int edgeNum = ptsw.size();
        if(kpts.size()==edgeNum && edgeNum >0)
        {
            for(unsigned int i=0;i<edgeNum;i++)
            {
                //EdgeReprojectionPoseOnly e(ptsw[i],K);
                //shared_ptr<VertexPose> vertexCam(new VertexPose());
                std::shared_ptr<EdgeReprojectionPoseOnly> pe(new EdgeReprojectionPoseOnly(ptsw[i],K));
                psolver->AddEdge(pe);
            }
        }
        else
        {
            std::cout<<"wrong edge num"<<std::endl;
        }


        //add vertex
       Sophus::SE3 T(Tcw.block(0,0,3,3),Tcw.block(0,3,3,1));
        Vec6 kexi = T.log();

        std::shared_ptr<VertexPose> pv(new VertexPose());
        pv->SetFixed(false);
        pv->SetParameters(kexi);

        psolver->AddVertex(pv);

        psolver->Solve();

        Vec6 kexi_new;

        if(kexi_new.norm()>100)
        {
            return false;
        }

        kexi_new=pv->Parameters();
        Sophus::SE3 Tnew = Sophus::SE3::exp(kexi_new);

        TTT = Tnew.matrix();

    }
    else
    {
        std::cout<<"wrong optimization type"<<std::endl;
    }
}
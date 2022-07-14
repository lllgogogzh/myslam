#include "ProblemSolver.h"


ProblemSolver::ProblemSolver()
{
    ;
}


bool ProblemSolver::AddVertex(std::shared_ptr<Vertex> vertex)
{
    if(mVerticies.find(vertex->Id())!=mVerticies.end())
    {
        //we can find such ID for current vertex
        //this is not good
        std::cout<<"This Vertex "<<vertex->Id()<<" has been exsit"<<std::endl;
        return false;
    }
    else
    {
        mVerticies.insert(std::pair<unsigned long,std::shared_ptr<Vertex>>(vertex->Id(),vertex));
        return true;
    }
}

bool ProblemSolver::AddEdge(std::shared_ptr<Edge> edge)
{
    if(mEdges.find(edge->Id())!=mEdges.end())
    {
        //we can find such ID for current vertex
        //this is not good
        std::cout<<"This edge "<<edge->Id()<<" has been exsit"<<std::endl;
        return false;
    }
    else
    {
        mEdges.insert(std::pair<unsigned long,std::shared_ptr<Edge>>(edge->Id(),edge));
        return true;
    }
}



bool ProblemSolver::DeleteVertex(std::shared_ptr<Vertex> vertex)
{
    return true;
}
bool ProblemSolver::DeleteEdge(std::shared_ptr<Edge> edge)
{
    return true;
}





bool ProblemSolver::Solve()
{
    //for debug : print some info
    Initialize();
    PrintInfomation();

    int NNNNN = 10;
    for(int i=0;i<NNNNN;i++)
    {
        MakeHessian();
        mDelta_x = (mHessian+0.001*Mat66::Identity()).inverse()*mb;
        
        //int VertexNum = mVerticies.size();

        for(auto ver:mVerticies)
        {
            VecX deltax = mDelta_x.block(ver.second->Id()*ver.second->LocalDimension(),0,ver.second->LocalDimension(),1);
            ver.second->Plus(deltax);
        }
    }


    return true;
}

void ProblemSolver::PrintInfomation()
{
    std::cout<<"------------------print info---------------------"<<std::endl;
    std::cout<<"Vertex num:"<<mVerticies.size()<<std::endl;
    std::cout<<"Edge num:"<<mEdges.size()<<std::endl;
    std::cout<<"------------------end print info---------------------"<<std::endl;
}

void ProblemSolver::MakeHessian()
{
    int order = mnHessianOrder;
    MatXX H(MatXX::Zero(order, order));
    MatXX b(MatXX::Zero(order,1));

    for(auto edge:mEdges)
    {
        if(edge.second)
        {
            //this edge is valid
            std::vector<std::shared_ptr<Vertex>> verticies = edge.second->Verticies();
            int edgeNumVertex = edge.second->NumVertices();

            if(edgeNumVertex == 1)
            {
                //unary edge
                if(verticies[0]->TypeInfo()==std::string("VertexPose"))
                {
                    edge.second->ComputeJacobians();
                    edge.second->ComputeResidual();

                    std::vector<MatXX> jac = edge.second->Jacobians();

                    Mat66 smallH = jac[0].transpose()*jac[0];
                    Vec6 smallb = -jac[0].transpose()*edge.second->Residual();

                    int vid=verticies[0]->Id();
                    int vlocal = verticies[0]->LocalDimension();


                    H.block(vid*vlocal,vid*vlocal,vlocal,vlocal) += smallH;
                    b.block(vid*vlocal,0,vlocal,1) += smallb;
                }
            }
            else if(edgeNumVertex == 2)
            {
                //binary edge
                //TODO : add binary edge and make Hessian
            }
        }
    }


    mb = b;
    mHessian = H;

}

void ProblemSolver::Initialize()
{
    mnHessianOrder =0;
    mnVertexNum = 0;
    mnPoseVertexNum = 0;
    mnEdgeNum = 0;

    if(mVerticies.size()>0)
    {
        for(auto vertex : mVerticies)
        {
            if(vertex.second)
            {
                mnHessianOrder+= vertex.second->LocalDimension();
                mnVertexNum++;

                if(vertex.second->TypeInfo()==std::string("VertexPose"))
                {
                    mnPoseVertexNum++;
                }
                else if(vertex.second->TypeInfo()==std::string("VertexXYZ"))
                {
                    mnXYZVertexNum++;
                }
            }
        }
    }

    /*if(mEdges.size()>0)
    {
        for(auto edge:mEdges)
        {
            if(edge.second)
            {
                mnEdgeNum++;
            }
        }
    }*/
}



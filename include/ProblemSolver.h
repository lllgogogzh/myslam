#ifndef PROBLEMSOLVER_H
#define PROBLEMSOLVER_H

#include <Vertex.h>
#include <Edge.h>
#include <VertexPose.h>
#include <VertexXYZ.h>
#include <EdgeReprojectionPoseOnly.h>
#include <EdgeReprojectionXYZ.h>

#include <iostream>
#include <map>
#include <vector>
#include <unordered_map>
#include <memory>

class ProblemSolver
{
    public:
        ProblemSolver();
        ~ProblemSolver(){;};


        //    typedef std::unordered_map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
        typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
        typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
        typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

        // base class 's pointer
        bool AddVertex(std::shared_ptr<Vertex> vertex);
        bool AddEdge(std::shared_ptr<Edge> edge);
        bool DeleteVertex(std::shared_ptr<Vertex> vertex);
        bool DeleteEdge(std::shared_ptr<Edge> edge);


        //the solver
        bool Solve();

        //print this problem info ,such as vertex number.
        void PrintInfomation();

        //construct the Hessian Matrix
        void MakeHessian();

        //init
        void Initialize();


        int mnVertexNum;
        int mnEdgeNum;

        HashVertex mVerticies;
        HashEdge mEdges;

        // 由vertex id查询edge
        HashVertexIdToEdge mVerticiesToEdges;

        public:
            //for nonlinear optimization
            // 整个信息矩阵
            MatXX mHessian;
            VecX mb;
            VecX mDelta_x;

            int mnPoseVertexNum;
            int mnXYZVertexNum;

            int mnHessianOrder;

};






#endif
#include "Frame.h"
using namespace GzhSLAM;

Frame::Frame(cv::Mat &color,cv::Mat &depth,bool isExtractORB,double time)
{
    mColorImgMat = color;
    mDepthImgMat = depth;
    mdTime=time;
    mTcw = cv::Mat::eye(4,4,CV_64F);
    mFASTDector = cv::ORB::create();
    mDescriptionExtractor = cv::ORB::create();
    mDescriptorMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    if(isExtractORB)
    {
        mFASTDector->detect(mColorImgMat,mvKeyPoints,cv::Mat());
        mDescriptionExtractor->compute(mColorImgMat,mvKeyPoints,mDescriptions);
        if(DEBUG==1)
        {
            ROS_INFO("Num of Keypoints:%ld ",mvKeyPoints.size());
        }
        //getKeyPointsDepth();
    }
}

bool Frame::isOverWide(double x,double y,const cv::Mat depth)
{
    int cols = depth.cols;
    int rows =depth.rows;

    if(x<=0||y<=0||x>=cols||y>=rows)
        return true;
    else
        return false;
}

void Frame::getKeyPointsDepth()
{
    if(!mDepthImgMat.empty()&&mvKeyPoints.size()>0)
    {
        for(auto it_Kpts : mvKeyPoints)
        {
            double x=it_Kpts.pt.x;
            double y=it_Kpts.pt.y;
            double depthScale = 5000.0;

            if(isOverWide(x,y,mDepthImgMat))
            {
                continue;
            }

            unsigned int d = mDepthImgMat.at<uchar>(y,x);
            bool baddepth=true;
            if(d==0)
            {
                //int col = mDepthImgMat.cols;
                //int row = mDepthImgMat.rows;
                for(int i=0;i<8;i++)
                {
                    double xt = x+EightDirection[i][0];
                    double yt = y+EightDirection[i][1];
                    if(isOverWide(xt,yt,mDepthImgMat))
                    {
                        continue;
                    }

                    double dt = mDepthImgMat.at<uchar>(yt,xt);
                    if(dt!=0)
                    {
                        baddepth = false;
                        d=dt;
                        break;
                    }
                }
            }
            else
            {
                baddepth = false;
            }
            
            if(!baddepth)
            {
                //gooddepth
                double realdepth = (double)d/depthScale;

            //TODO: pixel to Camera + recover depth + construct Camera in 3D
                recoverCameraPoints3D(x,y,realdepth);
            }
        }

        //std::cout<<"size"<<mvKeyPoints3D.size()<<std::endl;
    }
}

void Frame::recoverCameraPoints3D(const double x,const double y,double d)
{
    Convert convertOperator;

    Eigen::Vector3d cameraPts;
    convertOperator.Pixel2Camera(Eigen::Vector2d(x,y),cameraPts,d);
    
    Eigen::Vector3d worldPts;
    convertOperator.Camera2World(cameraPts,worldPts,mTcw);
    cv::Point3d p3d(worldPts(0),worldPts(1),worldPts(2));

    mvKeyPoints3D.push_back(p3d);
}





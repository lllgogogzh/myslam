#include"VisualOdom.h"

using namespace GzhSLAM;
using namespace std;



VisualOdom::VisualOdom()
{
    mnVOStatus = INITIALIZE;
    mDescriptorMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    mpConverter = std::make_shared<Convert>();
    mTcw=cv::Mat::eye(4,4,CV_64F);
}

VisualOdom::VisualOdom(ros::NodeHandle &n)
{
    mnVOStatus = INITIALIZE;
    mDescriptorMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    mpConverter = std::make_shared<Convert>();
    mTcw=cv::Mat::eye(4,4,CV_64F);
    nh=n;
    posePub = nh.advertise<nav_msgs::Odometry>("/myslam/odom",1,false);
    pubOdom = nav_msgs::Odometry();
}


void VisualOdom::TrackRGBD(cv::Mat &colorImg,cv::Mat &depthImg,double t)
{
    //frontend of visual odom
    if(mnVOStatus == VisualOdom::INITIALIZE)
    {
        //init
        //ROS_INFO("init");

        mpCurrentFrame = std::make_shared<Frame>(colorImg,depthImg,true,t);
        //cout<<mpCurrentFrame->mTcw;
        mpPrevFrame = mpCurrentFrame;
        mTcw=mpCurrentFrame->mTcw;
        mnVOStatus = VisualOdom::NORMAL;
        if(DEBUG==1)
        {
            cv::imshow("1",mpCurrentFrame->mColorImgMat);
            cv::waitKey(0);
        }
    }
    else if(mnVOStatus == VisualOdom::NORMAL)
    {
        //ROS_INFO("normal");
        //TODO: track by PnP
        //TODO: match

        mpCurrentFrame = std::make_shared<Frame>(colorImg,depthImg,true,t);
        FeatureMatch();
        RecoverDepth();
        TrackByPnP();
        Optimization();
        PubPoseOnce();
        mpPrevFrame = mpCurrentFrame;
        cv::waitKey(5);
    }
    else
    {
        //error
    }
}

void VisualOdom::TrackByPnP()
{
    /*if(mvGoodDMatchPoints.size()>0)
    {
        cout<<mvGoodDMatchPoints.size()<<endl;
        int nMatchesNum = mvGoodDMatchPoints.size();
        std::vector<cv::Point3d> vPtsInC = mpPrevFrame->mvKeyPoints3D;
        std::vector<cv::KeyPoint> vKeyPts = mpCurrentFrame->mvKeyPoints;

        mvPointsInWorld.clear();
        mvKeyPointsInCurFrame.clear();

        int lastIndex=-1;
        for(int i=0;i<nMatchesNum;i++)
        {
            cv::DMatch dmatch= mvGoodDMatchPoints[i];
            if(lastIndex == dmatch.queryIdx)
            {
                continue;
            }

            //dmatch.queryIdx
            cv::Point3d ptInW;
            cv::Mat_<double> pw=(cv::Mat_<double>(4,1)<<0.0,0.0,0.0,0.0);
            cv::Mat_<double> pc=(cv::Mat_<double>(4,1)<<vPtsInC[dmatch.queryIdx].x,vPtsInC[dmatch.queryIdx].y,vPtsInC[dmatch.queryIdx].z,1.0);
            
            //TODO
            //cnmcnmcnm
            //:::: huan yi ge fang shi qiu Twc
            
            //mTcw = cv::Mat::eye(4,4,CV_64F);
            pw = mTcw*pc;
            cout<<"pw"<<pw<<endl;
            mvPointsInWorld.push_back(cv::Point3d(pw.at<double>(0,0),pw.at<double>(1,0),pw.at<double>(2,0)));
            //mvKeyPointsInCurFrame.push_back(cv::Point2d(vKeyPts[dmatch.trainIdx].pt.x,vKeyPts[dmatch.trainIdx].pt.y));
            mvKeyPointsInCurFrame.push_back(vKeyPts[dmatch.trainIdx].pt);
            //ptInW = mTcw.colRange(3).rowRange(3).inv()*vPtsInC[dmatch.queryIdx];
            lastIndex = dmatch.queryIdx;

        }

        if(mvKeyPointsInCurFrame.size()==mvPointsInWorld.size())
        {
            cv::Mat Rvec,tvec,Inliers,mK,R;
            cv::eigen2cv(mpConverter->mK, mK);
            cv::solvePnPRansac(mvPointsInWorld,mvKeyPointsInCurFrame,mK,cv::Mat(),Rvec,tvec,false,300,(8.0F),0.99,Inliers,0);
            cv::Rodrigues(Rvec, R);//transfer vector(R) to rotatx matrix R
            
            cv::Mat_<double> Rt = (cv::Mat_<double>(4, 4) << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),tvec.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),tvec.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),tvec.at<double>(2,0),
            0, 0, 0, 1);

            mTcw = Rt.clone();
            cout<<mK<<endl;
            cout<<tvec<<endl;
            //cout<<R<<endl;
            //cout<<mTcw<<endl;
        }
        else
        {
            ROS_ERROR("PnP Wrong; Points In World num is:%d , but KeyPoints num is :%d",mvPointsInWorld.size(),mvKeyPointsInCurFrame.size());
        }
        
    }*/
    
    if(mvKeyPointsInCurFrame.size()==mvPointsInWorld.size())
    {
        cv::Mat Rvec,tvec,Inliers,mK,R;
        cv::eigen2cv(mpConverter->mK, mK);
        cv::solvePnPRansac(mvPointsInWorld,mvKeyPointsInCurFrame,mK,cv::Mat(),Rvec,tvec,false,300,(8.0F),0.99,Inliers,0);
        cv::Rodrigues(Rvec, R);//transfer vector(R) to rotatx matrix R
        
        cv::Mat_<double> Rt = (cv::Mat_<double>(4, 4) << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),tvec.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),tvec.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),tvec.at<double>(2,0),
        0, 0, 0, 1);

        mTcw = Rt.clone();
        //cout<<mK<<endl;
        //cout<<tvec<<endl;
        //cout<<R<<endl;
        //cout<<mTcw<<endl;
    }
    else
    {
        ROS_ERROR("PnP Wrong; Points In World num is:%d , but KeyPoints num is :%d",mvPointsInWorld.size(),mvKeyPointsInCurFrame.size());
    }
}

void VisualOdom::FeatureMatch()
{
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    //BFMatcher matcher ( NORM_HAMMING );
    mvDMatchPoints.clear();
    mDescriptorMatcher->match( mpPrevFrame->mDescriptions, mpCurrentFrame->mDescriptions, mvDMatchPoints );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < mpPrevFrame->mDescriptions.rows; i++ )
    {
        double dist = mvDMatchPoints[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    mvGoodDMatchPoints.clear();
    for ( int i = 0; i < mpPrevFrame->mDescriptions.rows; i++ )
    {
        if ( mvDMatchPoints[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            mvGoodDMatchPoints.push_back ( mvDMatchPoints[i] );
        }
    }

    cv::Mat img_match;
    cv::Mat img_goodmatch;
    //drawMatches ( mpPrevFrame->mColorImgMat, mpPrevFrame->mvKeyPoints, , keypoints_2, matches, img_match );
    cv::drawMatches ( mpPrevFrame->mColorImgMat, mpPrevFrame->mvKeyPoints,  mpCurrentFrame->mColorImgMat, 
                                    mpCurrentFrame->mvKeyPoints,mvGoodDMatchPoints,  img_goodmatch );
    //imshow ( "所有匹配点对", img_match );
    cv::imshow ( "123", img_goodmatch );
    //cout<<"1"<<endl;
}

void VisualOdom::RecoverDepth()
{
    vector<cv::DMatch> goodmatch = mvGoodDMatchPoints;
    vector<cv::KeyPoint> preKeypts = mpPrevFrame->mvKeyPoints;

    mvPointsInWorld.clear();
    mvKeyPointsInCurFrame.clear();

    for ( cv::DMatch m:goodmatch )
    {
        double x=preKeypts[m.queryIdx].pt.x;
        double y=preKeypts[m.queryIdx].pt.y;
        //ushort d1 = refDepthImg.ptr<unsigned short> ( int ( refKeyPoint[m.queryIdx].pt.y ) ) [ int ( refKeyPoint[m.queryIdx].pt.x ) ];
        //ushort d2 = curDepthImg.ptr<unsigned short> ( int ( curKeyPoint[m.trainIdx].pt.y ) ) [ int ( curKeyPoint[m.trainIdx].pt.x ) ];
        ushort d1 = mpPrevFrame->mDepthImgMat.at<unsigned short>(int(mpPrevFrame->mvKeyPoints[m.queryIdx].pt.y),int ( mpPrevFrame->mvKeyPoints[m.queryIdx].pt.x ));
        //ushort d2 = curDepthImg.at<unsigned short>(int(curKeyPoint[m.trainIdx].pt.y),int ( curKeyPoint[m.trainIdx].pt.x ));
        //cout<<d1<<","<<d11<<endl<<d2<<","<<d22<<endl;
        if ( d1==0 )   // bad depth
            continue;
        Eigen::Vector3d ptsc,ptsw;

        double dd1 = double ( d1 ) /5000.0;
        mpConverter->Pixel2Camera ( Eigen::Vector2d(x,y), ptsc ,dd1);

        mpConverter->Camera2World(ptsc,ptsw,mTcw);

        mvPointsInWorld.push_back ( cv::Point3d ( ptsw(0), ptsw(1),ptsw(2) ) );
        mvKeyPointsInCurFrame.push_back ( cv::Point2d (mpCurrentFrame->mvKeyPoints[m.trainIdx].pt.x,mpCurrentFrame->mvKeyPoints[m.trainIdx].pt.y) );
    }
}

void VisualOdom::PubPoseOnce()
{
    //init
    pubOdom = nav_msgs::Odometry();
    pubOdom.header.frame_id = "camera";
    pubOdom.pose.pose.position.x=mTcw.at<double>(0,3);
    pubOdom.pose.pose.position.x=mTcw.at<double>(1,3);
    pubOdom.pose.pose.position.x=mTcw.at<double>(2,3);

    posePub.publish(pubOdom);

    //创建一个tf对象
    tf::Transform transform;
    //设置tf的位移量
    transform.setOrigin( tf::Vector3(mTcw.at<double>(2,3), -mTcw.at<double>(0,3), mTcw.at<double>(1,3)));
    //初始化一个四元数变量
    tf::Quaternion q;
    //设置俯仰角
    q.setRPY(0.0,0.0,0.0);
    //将四元数存入tf对象中，tf只能存储四元数。
    transform.setRotation(q);
    //"robot" 是父坐标，"world"是子坐标
    //转换关系是从robot坐标系转到world坐标系
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world","camera"));
    //ROS_INFO("The transform has broadcasted!");
    //ros::Duration(0.002).sleep();

}

void VisualOdom::Optimization()
{
    Optimizer solver;
    Eigen::Matrix4d T;
    cv::cv2eigen(mTcw,T);

    std::vector<Eigen::Vector2d> kpts;
    std::vector<Eigen::Vector3d> ptsw;
    for(int i=0;i<mvKeyPointsInCurFrame.size();i++)
    {
        Eigen::Vector2d k(mvKeyPointsInCurFrame[i].x,mvKeyPointsInCurFrame[i].y);
        Eigen::Vector3d pw(mvPointsInWorld[i].x,mvPointsInWorld[i].y,mvPointsInWorld[i].z);

        kpts.push_back(k);
        ptsw.push_back(pw);
    }
    Eigen::Matrix4d eTcw;
    bool isGoodPose;
    isGoodPose = solver.OptimizePoseOnly(Optimizer::ONLYPOSE,T,kpts,ptsw,mpConverter->mK,eTcw);
    cout<<eTcw<<endl;
    if(isGoodPose)
    {
        cv::Mat Tcw_new;
        cv::eigen2cv(eTcw,Tcw_new);
        mTcw = Tcw_new.clone();
    }
}






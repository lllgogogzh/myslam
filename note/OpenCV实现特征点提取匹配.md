## OpenCV实现特征点提取匹配

### 一、理论概述

### 二、代码分析

在Frame类构造函数中，包括FAST角点提取以及描述子提取的过程

```c++
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
```

在VisualOdom类构造函数中，包括特征点匹配的过程

```c++
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
    cv::drawMatches ( mpPrevFrame->mColorImgMat, mpPrevFrame->mvKeyPoints,  mpCurrentFrame->mColorImgMat, 
                                    mpCurrentFrame->mvKeyPoints,mvGoodDMatchPoints,  img_goodmatch );
    cv::imshow ( "123", img_goodmatch );
    //cout<<"1"<<endl;
}
```

### 三、KeyPoints类

### 四、DMatch类
















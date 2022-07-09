## OpenCV实现PnP

### 一、理论概述

### 二、代码



```c++
void VisualOdom::TrackByPnP()
{
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

    }
    else
    {
        ROS_ERROR("PnP Wrong; Points In World num is:%d , but KeyPoints num is :%d",mvPointsInWorld.size(),mvKeyPointsInCurFrame.size());
    }
}
```





### 三、利用深度恢复特征点3D坐标

#### i. 各种坐标系转换

#### ii. 恢复3D坐标


























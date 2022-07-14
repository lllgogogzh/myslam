## tf树的发布——基于C++



```c++
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
```

发表速率取决于实际算法运行速率。
























//
// Created by minieye on 2021/6/2.
//

#ifndef FUSION_FUSION_NODE_H
#define FUSION_FUSION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <iostream>

class FusionNode {
public:
    FusionNode(ros::NodeHandle &nh);

    ~FusionNode() {}

    void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msgs);

    void GpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msgs);

private:
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    ros::Publisher path_pub_;
    ros::Publisher odom_pub_;

    nav_msgs::Path nav_path_;
};

#endif //FUSION_FUSION_NODE_H

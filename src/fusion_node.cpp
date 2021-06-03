//
// Created by minieye on 2021/6/2.
//

#include "fusion_node.h"
#include <ros/ros.h>

FusionNode::FusionNode(ros::NodeHandle &nh) {
    // ROS sub & pub
    std::string topic_imu = "/imu/data";
    std::string topic_gps = "/fix";

    imu_sub_ = nh.subscribe(topic_imu, 10, &FusionNode::ImuCallback, this);
    gps_sub_ = nh.subscribe(topic_gps, 10, &FusionNode::GpsCallback, this);

    path_pub_ = nh.advertise<nav_msgs::Path>("nav_path", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("nav_odom", 10);
}

void FusionNode::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msgs) {
    ROS_INFO("imu callback");
}

void FusionNode::GpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msgs) {
    ROS_INFO("gps callback");
}

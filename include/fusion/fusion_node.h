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
#include <fstream>
#include <deque>

#include "kalman_filter.h"

using namespace lu;

class FusionNode {
public:
    FusionNode(ros::NodeHandle &nh);

    ~FusionNode();

    void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msgs);

    void GpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msgs);

    // bool InitRot(Eigen::Matrix3d &R);
    bool InitRot(Sophus::SO3d &R);

    void PublishState();

private:
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;

    ros::Publisher path_pub_;
    ros::Publisher odom_pub_;

    ros::Publisher debug_pub_acc_;
    ros::Publisher debug_pub_gyr_;

    nav_msgs::Path nav_path_;

    std::unique_ptr<KalmanFilter> kf_ptr_;

    Eigen::Matrix3d R_CI;

    // init
    bool initialized_ = false;
    const int kImuBufSize = 100;
    std::deque<ImuDataConstPtr> imu_buf_;
    ImuDataConstPtr last_imu_ptr_;
    Eigen::Vector3d init_lla_;
    Eigen::Vector3d I_p_gps;

    // log files
    std::ofstream file_gps_;
    std::ofstream file_state_;

};

#endif //FUSION_FUSION_NODE_H

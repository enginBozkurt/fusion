//
// Created by minieye on 2021/6/2.
//

#include "fusion_node.h"
#include <ros/ros.h>

FusionNode::FusionNode(ros::NodeHandle &nh) {
    kf_ptr_ = std::make_unique<KalmanFilter>();
    I_p_gps = Eigen::Vector3d(0., 0., 0.);

    // ROS sub & pub
    std::string topic_imu = "/imu/data";
    std::string topic_gps = "/fix";
    imu_sub_ = nh.subscribe(topic_imu, 10, &FusionNode::ImuCallback, this);
    gps_sub_ = nh.subscribe(topic_gps, 10, &FusionNode::GpsCallback, this);

    path_pub_ = nh.advertise<nav_msgs::Path>("nav_path", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("nav_odom", 10);

    // debug : display curve
    debug_pub_acc_ = nh.advertise<geometry_msgs::Vector3>("acc", 10);
    debug_pub_gyr_ = nh.advertise<geometry_msgs::Vector3>("gyr", 10);

    R_CI << 0, 1, 0, 0, 0, -1, -1, 0, 0;

    // log files
    file_gps_.open("fusion_gps.csv");
    file_state_.open("fusion_state.csv");
}

FusionNode::~FusionNode() {
    file_gps_.close();
    file_state_.close();
}

void FusionNode::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msgs) {
    // ROS_INFO("imu callback");
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msgs->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msgs->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msgs->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msgs->linear_acceleration.z;
    imu_data_ptr->gyr[0] = imu_msgs->angular_velocity.x;
    imu_data_ptr->gyr[1] = imu_msgs->angular_velocity.y;
    imu_data_ptr->gyr[2] = imu_msgs->angular_velocity.z;

    imu_data_ptr->acc = R_CI * imu_data_ptr->acc;
    imu_data_ptr->gyr = R_CI * imu_data_ptr->gyr;

    if (!initialized_) {
        imu_buf_.push_back(imu_data_ptr);
        if (imu_buf_.size() > kImuBufSize)
            imu_buf_.pop_front();
        return;
    }

    /// prediction
    kf_ptr_->Prediction(last_imu_ptr_, imu_data_ptr);

    // update imu data
    last_imu_ptr_ = imu_data_ptr;

    // publish and save state
    PublishState();
}

void FusionNode::GpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msgs) {
//    if (gps_msgs->status.status != 2) {
//        ROS_INFO("[%s] ERROR: Bad GPS Message!!!\n", __FUNCTION__);
//        return;
//    }

    // ROS_INFO("gps callback");
    GpsDataPtr gps_data_ptr = std::make_shared<GpsData>();
    gps_data_ptr->timestamp = gps_msgs->header.stamp.toSec();
    gps_data_ptr->lla[0] = gps_msgs->latitude;
    gps_data_ptr->lla[1] = gps_msgs->longitude;
    gps_data_ptr->lla[2] = gps_msgs->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msgs->position_covariance.data());

    if (!initialized_) {
        if (imu_buf_.size() < kImuBufSize) {
            ROS_INFO("[%s] ERROR: Not Enough IMU data for Initialization!!!", __FUNCTION__);
            return;
        }

        last_imu_ptr_ = imu_buf_.back();
        if (std::abs(gps_data_ptr->timestamp - last_imu_ptr_->timestamp) > 0.5) {
            ROS_INFO("[%s] ERROR: Gps and Imu timestamps are not synchronized!!!", __FUNCTION__);
            return;
        }

        kf_ptr_->state_ptr_->timestamp = last_imu_ptr_->timestamp;
        if (!InitRot(kf_ptr_->state_ptr_->R_))
            return;

        init_lla_ = gps_data_ptr->lla;

        /// test
        // kf_ptr_->state_ptr_->R_ = Eigen::Matrix3d::Identity();

        initialized_ = true;

        ROS_INFO("[%s] System initialized.\n", __FUNCTION__);
        return;
    }

    // convert WGS84 to ENU frame
    Eigen::Vector3d p_gps;
    lu::lla2enu(init_lla_, gps_data_ptr->lla, &p_gps);

    // residual
    const Eigen::Vector3d &p_GI = kf_ptr_->state_ptr_->p_;
    // const Eigen::Matrix3d &R_GI = kf_ptr_->state_ptr_->R_;
    const Eigen::Matrix3d &R_GI = kf_ptr_->state_ptr_->R_.matrix();

    // gps - (imu + imu_to_gps)
    Eigen::Vector3d residual = p_gps - (p_GI + R_GI * I_p_gps);     // GPS只观测位置信息p，需要转到同一坐标系下才能计算残差
    // 此处将IMU下的p_GI通过坐标系变换，转到了GPS下进行
    // p = p + dp
    // R = R * dR
    // R 残差如何计算？

    // jacobian
    // residual的()内容 对位置求导，第一项位置为单位阵，第二项为速度偏导为零，第三项对R_GI * I_p_gps偏导为-R[x]x
    Eigen::Matrix<double, 3, kStateDim> H = Eigen::Matrix<double, 3, kStateDim>::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(0,6) = - R_GI * lu::skew_matrix(I_p_gps);  // ???

    // measurement covariance
    const Eigen::Matrix3d &V = gps_data_ptr->cov;

    /// correction
    kf_ptr_->Correction(H, V, residual);

    // save gps lla
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data_ptr->timestamp << ", "
              << gps_data_ptr->lla[0] << ", " << gps_data_ptr->lla[1] << ", " << gps_data_ptr->lla[2]
              << std::endl;
}


//bool FusionNode::InitRot(Eigen::Matrix3d &R) {
bool FusionNode::InitRot(Sophus::SO3d &R) {
    // mean and std of imu acc
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buf_)
        sum_acc += imu_data->acc;
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf_.size();

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buf_)
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buf_.size()).cwiseSqrt();

    if (std_acc.maxCoeff() > 3.0)
        return false;

    // Compute rotation.
    // ref: https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp

    // Three axises of the ENU frame in the IMU frame.
    // z-axis
    const Eigen::Vector3d &z_axis = mean_acc.normalized();

    // x-axis
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d r_IG;
    r_IG.block<3, 1>(0, 0) = x_axis;
    r_IG.block<3, 1>(0, 1) = y_axis;
    r_IG.block<3, 1>(0, 2) = z_axis;

    // R = r_IG.transpose();
    R.matrix() = r_IG.transpose();
    return true;
}

void FusionNode::PublishState() {
    // debug msgs
//    ROS_INFO("acc: [%f, %f, %f], gyr: [%f, %f, %f]",
//             kf_ptr_->state_ptr_->acc_bias_.x(),
//             kf_ptr_->state_ptr_->acc_bias_.y(),
//             kf_ptr_->state_ptr_->acc_bias_.z(),
//             kf_ptr_->state_ptr_->gyr_bias_.x(),
//             kf_ptr_->state_ptr_->gyr_bias_.y(),
//             kf_ptr_->state_ptr_->gyr_bias_.z());

    geometry_msgs::Vector3 debug_mgs;
    debug_mgs.x = kf_ptr_->state_ptr_->acc_bias_.x();
    debug_mgs.y = kf_ptr_->state_ptr_->acc_bias_.y();
    debug_mgs.z = kf_ptr_->state_ptr_->acc_bias_.z();
    debug_pub_acc_.publish(debug_mgs);

    debug_mgs.x = kf_ptr_->state_ptr_->gyr_bias_.x();
    debug_mgs.y = kf_ptr_->state_ptr_->gyr_bias_.y();
    debug_mgs.z = kf_ptr_->state_ptr_->gyr_bias_.z();
    debug_pub_gyr_.publish(debug_mgs);

    // publish the odometry
    std::string fixed_id = "world";
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = fixed_id;
    odom_msg.header.stamp = ros::Time::now();

    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    // T_wb.linear() = kf_ptr_->state_ptr_->R_;
    T_wb.linear() = kf_ptr_->state_ptr_->R_.matrix();
    T_wb.translation() = kf_ptr_->state_ptr_->p_;

    tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
    tf::vectorEigenToMsg(kf_ptr_->state_ptr_->v_, odom_msg.twist.twist.linear);

    Eigen::Matrix3d P_pp = kf_ptr_->state_ptr_->cov.block<3,3>(0,0);
    Eigen::Matrix3d P_po = kf_ptr_->state_ptr_->cov.block<3,3>(0,6);
    Eigen::Matrix3d P_op = kf_ptr_->state_ptr_->cov.block<3,3>(6,0);
    Eigen::Matrix3d P_oo = kf_ptr_->state_ptr_->cov.block<3,3>(6,6);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;
    for (int i = 0; i < 36; ++i)
        odom_msg.pose.covariance[i] = P_imu_pose.data()[i];

    odom_pub_.publish(odom_msg);

    // publish the path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    nav_path_.header = pose_stamped.header;
    nav_path_.poses.push_back(pose_stamped);
    path_pub_.publish(nav_path_);

    // save state p q lla
    std::shared_ptr<State> kf_state(kf_ptr_->state_ptr_);
    Eigen::Vector3d lla;
    lu::enu2lla(init_lla_, kf_state->p_, &lla);  // convert ENU state to lla
    // const Eigen::Quaterniond q_GI(kf_state->R_);
    const Eigen::Quaterniond q_GI(kf_state->R_.matrix());
    file_state_ << std::fixed << std::setprecision(15)
                << kf_state->timestamp << ", "
                << kf_state->p_[0] << ", " << kf_state->p_[1] << ", " << kf_state->p_[2] << ", "
                << q_GI.x() << ", " << q_GI.y() << ", " << q_GI.z() << ", " << q_GI.w() << ", "
                << lla[0] << ", " << lla[1] << ", " << lla[2]
                << std::endl;
}
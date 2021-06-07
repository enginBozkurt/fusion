//
// Created by minieye on 2021/6/7.
//

#include "fusion/kalman_filter.h"

using namespace lu;

KalmanFilter::KalmanFilter(double acc_n, double gyr_n, double acc_w, double gyr_w)
    : acc_noise_(acc_n), gyr_noise_(gyr_n), acc_bias_noise_(acc_w), gyr_bias_noise_(gyr_w) {
    // 创建state指针
    state_ptr_ = std::make_shared<State>();

    // 协方差矩阵初始化
    const double kDegreeToRadian = M_PI / 180.;
    const double sigma_rp = 10. * kDegreeToRadian;
    const double sigma_yaw = 100. * kDegreeToRadian;

    state_ptr_->cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 100.;                 // position std: 10 m
    state_ptr_->cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 100.;                 // velocity std: 10 m/s
    state_ptr_->cov.block<2, 2>(6, 6) = Eigen::Matrix2d::Identity() * sigma_rp * sigma_rp;  // roll pitch std 10 degree
    state_ptr_->cov(8, 8) = sigma_yaw * sigma_yaw;                                                 // yaw std: 100 degree
    state_ptr_->cov.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.0004;               // Acc bias
    state_ptr_->cov.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.0004;             // Gyr bias
}

void KalmanFilter::Prediction(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
    const double dt = curr_imu->timestamp - last_imu->timestamp;
    const double dt2 = dt * dt;

    /// last state
    State last_state = *state_ptr_;

    /// curr state
    state_ptr_->timestamp = curr_imu->timestamp;
    // the nominal-state kinematics
    // p v R
    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.acc_bias_;
    const Eigen::Vector3d gyr_unbias = 0.5 * (last_imu->gyr + curr_imu->gyr) - last_state.gyr_bias_;
    const Eigen::Vector3d acc_nominal = last_state.R_ * acc_unbias + Eigen::Vector3d(0, 0, -kG);
    state_ptr_->p_ = last_state.p_ + last_state.v_ * dt + 0.5 * acc_nominal * dt2;
    state_ptr_->v_ = last_state.v_ + acc_nominal * dt;

    const Eigen::Vector3d delta_angle_axis = gyr_unbias * dt;
    double norm_delta_angle = delta_angle_axis.norm();
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    if (norm_delta_angle > __DBL_EPSILON__) {
        dR = Eigen::AngleAxisd(norm_delta_angle, delta_angle_axis.normalized()).toRotationMatrix();
        state_ptr_->R_ = last_state.R_ * dR;
    }

    // the error-state jacobian and perturbation matrices
    // Fx
    MatrixSD Fx = MatrixSD::Identity();
    Fx.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3,3>(3,6) = - state_ptr_->R_ * lu::skew_matrix(acc_unbias) * dt;
    Fx.block<3,3>(3,9) = - state_ptr_->R_ * dt;
    if (norm_delta_angle > __DBL_EPSILON__)
        Fx.block<3,3>(6,6) = dR.transpose();
    else
        Fx.block<3,3>(6,6).setIdentity();
    Fx.block<3,3>(6,12) = - Eigen::Matrix3d::Identity() * dt;

    // Fi
    Eigen::Matrix<double, kStateDim, kNoiseDim> Fi = Eigen::Matrix<double, kStateDim, kNoiseDim>::Zero();
    Fi.block<kNoiseDim, kNoiseDim>(3,0) = Eigen::Matrix<double, kNoiseDim, kNoiseDim>::Identity();

    // Qi
    Eigen::Matrix<double, kNoiseDim, kNoiseDim> Qi = Eigen::Matrix<double, kNoiseDim, kNoiseDim>::Zero();
    Qi.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * acc_noise_ * dt2;
    Qi.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * gyr_noise_ * dt2;
    Qi.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * acc_bias_noise_ * dt;
    Qi.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * gyr_bias_noise_ * dt;

    // update cov P
    state_ptr_->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();
}

void KalmanFilter::Correction(const Eigen::Matrix<double, 3, kStateDim> &H, const Eigen::Matrix3d &V,
                              const Eigen::Vector3d &residual) {
    /// step 1. observation of the error-state via filter correction
    // compute K
    const MatrixSD &P = state_ptr_->cov;
    const Eigen::Matrix3d S = H * P * H.transpose() + V;
    const Eigen::Matrix<double, kStateDim, 3> K = P * H.transpose() * S.inverse();

    // compute delta_x
    const Eigen::Matrix<double, kStateDim, 1> delta_x = K * residual;

    // update cov
    // todo : cov update function
    // state_ptr_->cov = (MatrixSD::Identity() - K * H) * P;
    const MatrixSD I_KH = MatrixSD::Identity() - K * H;
    state_ptr_->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();

    /// step 2. injection of the observed errors into the nominal-state
    // update to nominal-state
    state_ptr_->p_ += delta_x.block<3,1>(0,0);
    state_ptr_->v_ += delta_x.block<3,1>(3,0);

    const Eigen::Vector3d dR = delta_x.block<3,1>(6,0);
    if (dR.norm() > __DBL_EPSILON__)
        state_ptr_->R_ *= Eigen::AngleAxisd(dR.norm(), dR.normalized()).toRotationMatrix();

    state_ptr_->acc_bias_ += delta_x.block<3,1>(9,0);
    state_ptr_->gyr_bias_ += delta_x.block<3,1>(12,0);

    /// step 3. reset of the error-state


}
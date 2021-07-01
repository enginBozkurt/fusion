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

    // p_I_V, R_IV 初始化
    float pitch, yaw, roll;
    pitch = 0.0;
    yaw = 0.0;
    roll = 0.0;
    // pitch = -0.01;
    // yaw = 0.02;
    // roll = -0.014;

    float cx, ch, cl;
    cx = 0.0;
    ch = -1.2;
    cl = 1.90;


    double v = 0;   // 初始化时的speed
    state_ptr_->p_I_G.setZero();
    state_ptr_->b_a.setZero();
    state_ptr_->b_g.setZero();
    state_ptr_->p_I_V = Eigen::Vector3d(cx, ch, cl);
    state_ptr_->R_GI = Sophus::SO3d::exp(Eigen::Vector3d(-pitch, -yaw, -roll));
    state_ptr_->v_I_G = state_ptr_->R_GI.inverse() * state_ptr_->R_IV * Eigen::Vector3d(0, 0, v);
}

void KalmanFilter::Prediction(const ImuDataConstPtr& last_imu, const ImuDataConstPtr& curr_imu) {
    const double dt = curr_imu->timestamp - last_imu->timestamp;
    const double dt2 = dt * dt;

    /// last state
    State last_state = *state_ptr_;

    /// curr state
    state_ptr_->timestamp = curr_imu->timestamp;
    // the nominal-state kinematics
    // p v R
    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.b_a;
    const Eigen::Vector3d gyr_unbias = 0.5 * (last_imu->gyr + curr_imu->gyr) - last_state.b_g;
    const Eigen::Vector3d acc_nominal = last_state.R_GI * acc_unbias + Eigen::Vector3d(0, 0, -kG);
    state_ptr_->p_I_G = last_state.p_I_G + last_state.v_I_G * dt + 0.5 * acc_nominal * dt2;
    state_ptr_->v_I_G = last_state.v_I_G + acc_nominal * dt;
    state_ptr_->R_GI = state_ptr_->R_GI * Sophus::SO3d::exp(gyr_unbias * dt);

    // the error-state jacobian and perturbation matrices
    // Fx
    MatrixSD Fx = MatrixSD::Identity();
    Fx.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;                                  // p
    Fx.block<3,3>(3,6) = - state_ptr_->R_GI.matrix() * lu::skew_matrix(acc_unbias) * dt;    // v
    Fx.block<3,3>(3,9) = - state_ptr_->R_GI.matrix() * dt;
    Fx.block<3,3>(6,6) = Sophus::SO3d::exp(gyr_unbias * dt).matrix().transpose();           // R
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
#ifdef ODO
    Qi.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * 1e-6 * dt;
    Qi.block<3,3>(15,15) = Eigen::Matrix3d::Identity() * 1e-6 * dt;
#endif

    // update cov P
    state_ptr_->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();
}

//void KalmanFilter::Correction(const Eigen::Matrix<double, 3, kStateDim> &H, const Eigen::Matrix3d &V,
//                              const Eigen::Vector3d &residual) {
void KalmanFilter::Correction(const Eigen::MatrixXd &H, const Eigen::MatrixXd &V,
                              const Eigen::VectorXd &residual) {
    /// step 1. observation of the error-state via filter correction
    // compute K
    const MatrixSD &P = state_ptr_->cov;
    const Eigen::MatrixXd S = H * P * H.transpose() + V;
    const Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // compute delta_x
    const Eigen::MatrixXd delta_x = K * residual;

    // update cov
    // to do : cov update function
    // state_ptr_->cov = (MatrixSD::Identity() - K * H) * P;
    const Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(kStateDim, kStateDim) - K * H;
    state_ptr_->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();

    /// step 2. injection of the observed errors into the nominal-state
    // std::cout << "size of delta : " << delta_x.size() << std::endl;

    // update to nominal-state
    state_ptr_->p_I_G += delta_x.block<3,1>(0, 0);
    state_ptr_->v_I_G += delta_x.block<3,1>(3, 0);
    state_ptr_->R_GI = state_ptr_->R_GI * Sophus::SO3d::exp(delta_x.block<3,1>(6, 0));
    state_ptr_->b_a += delta_x.block<3,1>(9, 0);
    state_ptr_->b_g += delta_x.block<3,1>(12, 0);
#ifdef ODO
    state_ptr_->p_I_V += delta_x.block<3,1>(15, 0);
    state_ptr_->R_IV = state_ptr_->R_IV * Sophus::SO3d::exp(delta_x.block<3,1>(18, 0));
#endif

    // step 3. reset of the error-state
}
//
// Created by minieye on 2021/6/7.
//

#ifndef FUSION_KALMAN_FILTER_H
#define FUSION_KALMAN_FILTER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <se3.hpp>
#include <memory>

#include "utils.h"

namespace lu {

    constexpr int kStateDim = 15;
    constexpr int kNoiseDim = 12;
    constexpr double kG = 9.81007;

    using MatrixSD = Eigen::Matrix<double, kStateDim, kStateDim>;

    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MatrixSD cov;

        Eigen::Vector3d p_;
        Eigen::Vector3d v_;
        Eigen::Matrix3d R_;
        // Sophus::SO3f R_ = Sophus::SO3f();				 //rotation of imu

        Eigen::Quaterniond q_;

        Eigen::Vector3d acc_bias_;
        Eigen::Vector3d gyr_bias_;

        double timestamp;

        State() {
            cov.setZero();
            p_.setZero();
            v_.setZero();
            R_.setZero();
            // R_.matrix().setZero();
            acc_bias_.setZero();
            gyr_bias_.setZero();
        }
    };

    class KalmanFilter {
    public:
        KalmanFilter(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8);

        ~KalmanFilter(){};

        void Prediction(const ImuDataConstPtr& last_imu, const ImuDataConstPtr& curr_imu);

        void Correction(const Eigen::Matrix<double, 3, kStateDim> &H,
                        const Eigen::Matrix3d &V,
                        const Eigen::Vector3d &residual);

        std::shared_ptr<State> state_ptr_;

    private:
        double acc_noise_;
        double gyr_noise_;
        double acc_bias_noise_;
        double gyr_bias_noise_;
    };
}

#endif //FUSION_KALMAN_FILTER_H

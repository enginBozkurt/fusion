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

#define ODO

namespace lu {

#ifndef ODO
    constexpr int kStateDim = 15;
    constexpr int kNoiseDim = 12;

#else
    constexpr int kStateDim = 21;
    constexpr int kNoiseDim = 18;
#endif

    constexpr double kG = 9.81007;

    using MatrixSD = Eigen::Matrix<double, kStateDim, kStateDim>;

    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MatrixSD cov;

        Eigen::Vector3d p_I_G;
        Eigen::Vector3d v_I_G;
        Sophus::SO3d R_GI;				 //rotation of imu
        Eigen::Vector3d b_a;
        Eigen::Vector3d b_g;

        Sophus::SO3d R_IV;
        Eigen::Vector3d p_I_V;

        double timestamp;

        State() {
            cov.setZero();
            p_I_G.setZero();
            v_I_G.setZero();
            R_GI.matrix().setZero();
            b_a.setZero();
            b_g.setZero();

            R_IV.matrix().setZero();
            p_I_V.setZero();
        }
    };

    class KalmanFilter {
    public:
        KalmanFilter(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8);

        ~KalmanFilter(){};

        void Prediction(const ImuDataConstPtr& last_imu, const ImuDataConstPtr& curr_imu);

        void Correction(const Eigen::MatrixXd &H,
                        const Eigen::MatrixXd &V,
                        const Eigen::VectorXd &residual);

//        void Filter::measurementUpdate(const Eigen::VectorXf &r, Eigen::MatrixXf &H,
//                                       const Eigen::MatrixXf &R, const std::string &fun_name)

        std::shared_ptr<State> state_ptr_;

    private:
        double acc_noise_;
        double gyr_noise_;
        double acc_bias_noise_;
        double gyr_bias_noise_;
    };
}

#endif //FUSION_KALMAN_FILTER_H

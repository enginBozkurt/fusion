//
// Created by minieye on 2021/6/7.
//

#ifndef FUSION_UTILS_H
#define FUSION_UTILS_H

#include <Eigen/Core>
#include <GeographicLib/LocalCartesian.hpp>

namespace lu {

    struct ImuData {
        double timestamp = - 1.0;

        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
    };
    using ImuDataPtr = std::shared_ptr<ImuData>;
    using ImuDataConstPtr = std::shared_ptr<const ImuData>;

    struct SpeedData {
        double timestamp = -1.0;
        double v = - 1.0;
    };
    using SpeedDataPtr = std::shared_ptr<SpeedData>;
    using SpeedDataConstPtr = std::shared_ptr<const SpeedData>;

    struct YRData {
        double timestamp = - 1.0;
        double yr = -1.0;
    };
    using YRDataPtr = std::shared_ptr<YRData>;
    using YRDataConstPtr = std::shared_ptr<const YRData>;

    struct GpsData {
        double timestamp = - 1.0;

        Eigen::Vector3d lla = Eigen::Vector3d::Zero();  // Latitude in degree, longitude in degree, and altitude in meter
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();  // Covariance in m^2
    };
    using GpsDataPtr = std::shared_ptr<GpsData>;
    using GpsDataConstPtr = std::shared_ptr<const GpsData>;



    inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
        Eigen::Matrix3d w;
        w << 0., -v(2), v(1),
                v(2), 0., -v(0),
                -v(1), v(0), 0.;

        return w;
    }

    inline void lla2enu(const Eigen::Vector3d &init_lla,
                        const Eigen::Vector3d &point_lla,
                        Eigen::Vector3d *point_enu) {
        static GeographicLib::LocalCartesian local_cartesian;
        local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
        local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2),
                                point_enu->data()[0], point_enu->data()[1], point_enu->data()[2]);
    }

    inline void enu2lla(const Eigen::Vector3d &init_lla,
                        const Eigen::Vector3d &point_enu,
                        Eigen::Vector3d *point_lla) {
        static GeographicLib::LocalCartesian local_cartesian;
        local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
        local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2),
                                point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);
    }

}

#endif //FUSION_UTILS_H

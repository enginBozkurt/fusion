//
// Created by minieye on 2021/6/10.
//

#include <ros/ros.h>

#include<iostream>
#include <fstream>
#include<sstream>
#include<string>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#define ACCSCALE 1.0 / 4096.0
#define GYRSCALE 2000.0 * M_PI / 180.0 / 32768.0
#define Grav 9.7925

using namespace std;
//using namespace ros;

class SensorData {
public:
    SensorData(ros::NodeHandle &nh);

    ~SensorData(){}

    void PublishIMU();

    void PublishGPS();

    void PublishYawRate();

    void PublishSpeed();

    int frame_id_;
    double yaw_rate_;
    double speed_;
    ros::Time time_;
    sensor_msgs::Imu imu_;
    sensor_msgs::NavSatFix gps_;

    ros::Publisher imu_pub_;
    ros::Publisher gps_pub_;
};

//std::string topic_imu = "/imu/data";
//std::string topic_gps = "/fix";

SensorData::SensorData(ros::NodeHandle &nh) {
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);
    gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/fix", 10);

    gps_.position_covariance = {0.81, 0.0, 0.0,
                                0.0, 0.81, 0.0,
                                0.0, 0.0, 3.24};
}

void SensorData::PublishIMU() {
    imu_.header.stamp = time_;
    imu_.linear_acceleration.x *= ACCSCALE * Grav;
    imu_.linear_acceleration.y *= ACCSCALE * Grav;
    imu_.linear_acceleration.z *= ACCSCALE * Grav;
    imu_.angular_velocity.x *= GYRSCALE;
    imu_.angular_velocity.y *= GYRSCALE;
    imu_.angular_velocity.z *= GYRSCALE;

    imu_pub_.publish(imu_);
}

void SensorData::PublishGPS() {
    gps_.header.stamp = time_;
    gps_pub_.publish(gps_);
}

void SensorData::PublishYawRate() {

}

void SensorData::PublishSpeed() {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor");
    ros::NodeHandle nh;

    double rate;
    nh.param("play_rate", rate, 1.0);
    ros::Rate loop_rate(20 * rate);

    SensorData data(nh);

    int gps_rate_count = 0;

    ifstream file("/home/minieye/dataset/test_data/converted_data/log.txt");
    string line;
    while (getline(file, line)) {
        // prefix
        double time_s, time_ms;
        string fieldname;

        stringstream ss(line);
        ss >> time_s >> time_ms >> fieldname;

        // time
        data.time_.fromSec(time_s + time_ms * 1e-6);

        // 可改写成哈希表的形式 或者其他更优方式
        // camera
        if (strcmp(fieldname.c_str(),"cam_frame")==0) {
            ss >> data.frame_id_;
            loop_rate.sleep();
        }

            // IMU
        else if (strcmp(fieldname.c_str(),"Gsensor.1")==0) {
            ss >> data.imu_.linear_acceleration.x >> data.imu_.linear_acceleration.y >> data.imu_.linear_acceleration.z
               >> data.imu_.angular_velocity.x >> data.imu_.angular_velocity.y >> data.imu_.angular_velocity.z;
            data.PublishIMU();
        }

            // GPS
        else if (strcmp(fieldname.c_str(),"GPSRAW")==0) {
            ss >> data.gps_.latitude >> data.gps_.longitude >> data.gps_.altitude;

//            gps_rate_count++;
//            if (gps_rate_count == 2) {
//                data.PublishGPS();
//                gps_rate_count = 0;
//            }

            data.PublishGPS();
        }

            // yaw rate
        else if (strcmp(fieldname.c_str(),"YawRate")==0) {
            ss >> data.yaw_rate_;
            data.PublishYawRate();
        }

            // speed
        else if (strcmp(fieldname.c_str(),"speed")==0) {
            ss >> data.speed_;
            data.PublishSpeed();
        }
    }

    file.close();

    ros::spin();
    return 0;
}
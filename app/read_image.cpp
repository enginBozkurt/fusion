//
// Created by minieye on 2021/6/10.
//

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "dataset");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_pub = it.advertise("camera/image", 1);

    cv::VideoCapture capture("/home/minieye/dataset/test_data/converted_data/output.mp4");

    // 获取整个帧数
    double totalFrameNumber = capture.get( CV_CAP_PROP_FRAME_COUNT );
    ROS_INFO("total frames = %d", int(totalFrameNumber));

    // 获取帧率
    double rate = capture.get( CV_CAP_PROP_FPS );
    ROS_INFO("frame rate = %f", rate);

    double play_rate;
    nh.param("play_rate", play_rate, 1.0);
    ros::Rate loop_rate(rate * play_rate);  // 20

    while (ros::ok()) {
        cv::Mat frame;
        capture >> frame;

        if (frame.empty()) {
            break;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        img_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
//
// Created by minieye on 2021/6/2.
//

#include <ros/ros.h>
#include "fusion/fusion_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion");

    ros::NodeHandle nh;
    FusionNode fusion(nh);

    ros::spin();

    return 0;
}
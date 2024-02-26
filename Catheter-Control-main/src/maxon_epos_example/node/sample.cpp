/**
 * @file   sample
 * @brief  
 * @author 
 * @date    
 */

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "maxon_epos_msgs/MotorState.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("/maxon_bringup/all_position", 1000);
    std_msgs::Float32MultiArray msg = std_msgs::Float32MultiArray();
    msg.data.push_back(3000);
    msg.data.push_back(-2000);

    // ros::Publisher pub_all_motors = nh.advertise<std_msgs::Float32MultiArray>("~/get_all_state", 1000);
    // maxon_epos_msgs::MotorStates msg = maxon_epos_msgs::MotorStates();
    
    ros::Rate sleep_rate(50);
    while (ros::ok()) {
        pub.publish(msg);
        sleep_rate.sleep();
    }
    
    return 0;
}

#pragma once

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>

#include "RobotState.h"

using namespace std;

namespace robot {
class GazeboInterface {
   public:
    GazeboInterface(ros::NodeHandle& nh, string robot_name);

    void ctrl_update(double dt);
    void fbk_update();
    void send_cmd();

   private:
    // Robot state
    RobotState robot_state;

    // ROS publishers & subscribers
    ros::Publisher pub_joint_cmd[ACT_JOINTS];
    ros::Subscriber sub_joint_states[ACT_JOINTS];
    ros::Subscriber sub_torso_com_odom;
    ros::Subscriber sub_joy;

    // ROS messages
    unitree_legged_msgs::MotorCmd low_cmd;

    // Callback functions
    void torso_com_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);

    void left_hip_yaw_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void left_hip_abad_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void left_hip_pitch_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void left_knee_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void left_ankle_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);

    void right_hip_yaw_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void right_hip_abad_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void right_hip_pitch_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void right_knee_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void right_ankle_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
};
}  // namespace robot
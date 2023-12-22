#include "GazeboInterface.h"

using namespace std;

namespace robot {
GazeboInterface::GazeboInterface(ros::NodeHandle& nh, string robot_name) {
    // ROS publishers
    pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_hip_yaw_controller/command", 1);
    pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_hip_abad_controller/command", 1);
    pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_hip_pitch_controller/command", 1);
    pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_knee_controller/command", 1);
    pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_ankle_controller/command", 1);

    pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_hip_yaw_controller/command", 1);
    pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_hip_abad_controller/command", 1);
    pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_hip_pitch_controller/command", 1);
    pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_knee_controller/command", 1);
    pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_ankle_controller/command", 1);

    // ROS subscribers
    sub_joint_states[0] = nh.subscribe("/" + robot_name + "/left_hip_yaw_controller/state", 1, &GazeboInterface::left_hip_yaw_state_callback, this);
    sub_joint_states[1] = nh.subscribe("/" + robot_name + "/left_hip_abad_controller/state", 1, &GazeboInterface::left_hip_abad_state_callback, this);
    sub_joint_states[2] = nh.subscribe("/" + robot_name + "/left_hip_pitch_controller/state", 1, &GazeboInterface::left_hip_pitch_state_callback, this);
    sub_joint_states[3] = nh.subscribe("/" + robot_name + "/left_knee_controller/state", 1, &GazeboInterface::left_knee_state_callback, this);
    sub_joint_states[4] = nh.subscribe("/" + robot_name + "/left_ankle_controller/state", 1, &GazeboInterface::left_ankle_state_callback, this);

    sub_joint_states[5] = nh.subscribe("/" + robot_name + "/right_hip_yaw_controller/state", 1, &GazeboInterface::right_hip_yaw_state_callback, this);
    sub_joint_states[6] = nh.subscribe("/" + robot_name + "/right_hip_abad_controller/state", 1, &GazeboInterface::right_hip_abad_state_callback, this);
    sub_joint_states[7] = nh.subscribe("/" + robot_name + "/right_hip_pitch_controller/state", 1, &GazeboInterface::right_hip_pitch_state_callback, this);
    sub_joint_states[8] = nh.subscribe("/" + robot_name + "/right_knee_controller/state", 1, &GazeboInterface::right_knee_state_callback, this);
    sub_joint_states[9] = nh.subscribe("/" + robot_name + "/right_ankle_controller/state", 1, &GazeboInterface::right_ankle_state_callback, this);

    sub_torso_com_odom = nh.subscribe("/" + robot_name + "/torso_com_odom", 1, &GazeboInterface::torso_com_odom_callback, this);
    sub_joy = nh.subscribe("/joy", 1, &GazeboInterface::joy_callback, this);
}

void GazeboInterface::fbk_update() {
    cout << robot_state.joy_cmd.joy_vel_x << endl;
}

void GazeboInterface::torso_com_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    robot_state.fbk.torso_pos_world << odom_msg->pose.pose.position.x,
                                       odom_msg->pose.pose.position.y,
                                       odom_msg->pose.pose.position.z;
    robot_state.fbk.torso_quat = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                                    odom_msg->pose.pose.orientation.x,
                                                    odom_msg->pose.pose.orientation.y,
                                                    odom_msg->pose.pose.orientation.z);
    robot_state.fbk.torso_lin_vel_world << odom_msg->twist.twist.linear.x,
                                           odom_msg->twist.twist.linear.y,
                                           odom_msg->twist.twist.linear.z;
    robot_state.fbk.torso_ang_vel_world << odom_msg->twist.twist.angular.x,
                                           odom_msg->twist.twist.angular.y,
                                           odom_msg->twist.twist.angular.z;
}

void GazeboInterface::joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    robot_state.joy_cmd.joy_vel_x = joy_msg->axes[4] * robot_state.params.joy_vel_x_max;
    robot_state.joy_cmd.joy_vel_y = joy_msg->axes[3] * robot_state.params.joy_vel_y_max;
    robot_state.joy_cmd.joy_vel_z = joy_msg->axes[1] * robot_state.params.joy_vel_z_max;
    robot_state.joy_cmd.joy_roll_vel = -joy_msg->axes[6] * robot_state.params.joy_roll_vel_max;
    robot_state.joy_cmd.joy_pitch_vel = -joy_msg->axes[7] * robot_state.params.joy_pitch_vel_max;
    robot_state.joy_cmd.joy_yaw_vel = joy_msg->axes[0] * robot_state.params.joy_yaw_vel_max;
    if (joy_msg->buttons[4] == 1) {
        robot_state.joy_cmd.stop_control = true;
    }
}

void GazeboInterface::left_hip_yaw_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[0] = joint_state_msg.q;
    robot_state.fbk.joint_vel[0] = joint_state_msg.dq;
}

void GazeboInterface::left_hip_abad_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[1] = joint_state_msg.q;
    robot_state.fbk.joint_vel[1] = joint_state_msg.dq;
}

void GazeboInterface::left_hip_pitch_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[2] = joint_state_msg.q;
    robot_state.fbk.joint_vel[2] = joint_state_msg.dq;
}

void GazeboInterface::left_knee_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[3] = joint_state_msg.q;
    robot_state.fbk.joint_vel[3] = joint_state_msg.dq;
}

void GazeboInterface::left_ankle_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[4] = joint_state_msg.q;
    robot_state.fbk.joint_vel[4] = joint_state_msg.dq;
}

void GazeboInterface::right_hip_yaw_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[5] = joint_state_msg.q;
    robot_state.fbk.joint_vel[5] = joint_state_msg.dq;
}

void GazeboInterface::right_hip_abad_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[6] = joint_state_msg.q;
    robot_state.fbk.joint_vel[6] = joint_state_msg.dq;
}

void GazeboInterface::right_hip_pitch_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[7] = joint_state_msg.q;
    robot_state.fbk.joint_vel[7] = joint_state_msg.dq;
}

void GazeboInterface::right_knee_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[8] = joint_state_msg.q;
    robot_state.fbk.joint_vel[8] = joint_state_msg.dq;
}

void GazeboInterface::right_ankle_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[9] = joint_state_msg.q;
    robot_state.fbk.joint_vel[9] = joint_state_msg.dq;
}

}  // namespace robot
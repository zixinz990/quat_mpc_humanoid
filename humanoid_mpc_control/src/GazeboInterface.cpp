#include "GazeboInterface.h"
#include "Kinematics.h"

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
    sub_torso_imu = nh.subscribe("/" + robot_name + "/torso_imu", 1, &GazeboInterface::torso_imu_callback, this);
    sub_joy = nh.subscribe("/joy", 1, &GazeboInterface::joy_callback, this);

    robot_state.params.load(nh);
}

void GazeboInterface::ctrl_update() {
    robot_state.ctrl.joint_pos_d = robot_state.fbk.joint_pos;
    robot_state.ctrl.joint_vel_d = robot_state.fbk.joint_vel;
    // robot_state.ctrl.grf_d[2] = -robot_state.params.robot_mass * 9.81 / 2;
    // robot_state.ctrl.grf_d[5] = -robot_state.params.robot_mass * 9.81 / 2;

    robot_state.ctrl.joint_tau_d.block<LEG_DOF, 1>(0, 0) = robot_state.fbk.left_foot_jac.transpose() * robot_state.ctrl.grf_d.block<3, 1>(0, 0);
    robot_state.ctrl.joint_tau_d.block<LEG_DOF, 1>(LEG_DOF, 0) = robot_state.fbk.right_foot_jac.transpose() * robot_state.ctrl.grf_d.block<3, 1>(3, 0);
    send_cmd();
}

void GazeboInterface::fbk_update() {
    // Calculate rotation matrix
    robot_state.fbk.torso_rot_mat = robot_state.fbk.torso_quat.toRotationMatrix();
    robot_state.fbk.torso_euler = Utils::quat_to_euler(robot_state.fbk.torso_quat);
    robot_state.fbk.torso_rot_mat_z = Eigen::AngleAxisd(robot_state.fbk.torso_euler[2], Eigen::Vector3d::UnitZ());

    // Calculate foot Jacobian
    robot_state.fbk.left_foot_jac = Kinematics::cal_left_foot_jac(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(0, 0));
    robot_state.fbk.right_foot_jac = Kinematics::cal_right_foot_jac(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(LEG_DOF, 0));

    // Some coordinate transformation
    robot_state.fbk.torso_lin_vel_body = robot_state.fbk.torso_rot_mat.transpose() * robot_state.fbk.torso_lin_vel_world;
}

void GazeboInterface::send_cmd() {
    for (int i = 0; i < ACT_JOINTS; i++) {
        low_cmd.motorCmd[i].mode = 0x0A;
        low_cmd.motorCmd[i].q = 0;
        low_cmd.motorCmd[i].dq = 0;
        low_cmd.motorCmd[i].Kp = 0;
        low_cmd.motorCmd[i].Kd = 0;
        low_cmd.motorCmd[i].tau = robot_state.params.joint_kp * (robot_state.ctrl.joint_pos_d[i] - robot_state.fbk.joint_pos[i]) +
                                  robot_state.params.joint_kd * (robot_state.ctrl.joint_vel_d[i] - robot_state.fbk.joint_vel[i]) +
                                  robot_state.ctrl.joint_tau_d[i];
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
    }
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

void GazeboInterface::torso_imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    robot_state.fbk.torso_ang_vel_body << imu_msg->angular_velocity.x,
                                          imu_msg->angular_velocity.y,
                                          imu_msg->angular_velocity.z;
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

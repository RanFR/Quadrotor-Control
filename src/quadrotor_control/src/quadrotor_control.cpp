#include "quadrotor_control/quadrotor_control.hpp"

void QuadrotorControl::state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

void QuadrotorControl::init(const ros::NodeHandle& nh) {
    ROS_INFO("Quadrotor control start initialization.");

    // ros node handle
    nh_ = nh;

    // Ros subscriber and service client
    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &QuadrotorControl::state_callback, this);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    imu_freq_client_ = nh_.serviceClient<mavros_msgs::MessageInterval>("/mavros/set_message_interval");

    // main function
    run();
}

void QuadrotorControl::run() {
    // ros rate
    ros::Rate rate(20.0);

    // increase imu frequency
    mavros_msgs::MessageInterval imu_data_msgs, imu_data_raw_msgs;
    // imu/data_raw: 105, imu/data: 31
    imu_data_msgs.request.message_id = 31;
    imu_data_raw_msgs.request.message_id = 105;
    imu_data_msgs.request.message_rate = imu_data_raw_msgs.request.message_rate = 100.0;
    if (imu_freq_client_.call(imu_data_msgs) && imu_data_msgs.response.success) {
        ROS_INFO("IMU data frequency set to 100Hz");
    } else {
        ROS_WARN("Failed to increase IMU data frequency");
    }
    if (imu_freq_client_.call(imu_data_raw_msgs) && imu_data_raw_msgs.response.success) {
        ROS_INFO("IMU data raw frequency set to 100Hz");
    } else {
        ROS_WARN("Failed to increase IMU data raw frequency");
    }

    // check connection
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode set_mode_offboard;
    set_mode_offboard.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // offboard and arm, loop execution
    while (ros::ok()) {
        if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client_.call(set_mode_offboard) && set_mode_offboard.response.mode_sent) {
                ROS_INFO("Offboard mode enabled.");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed.");
                    break; // exit the loop
                }
                last_request = ros::Time::now();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Quadrotor control initialization complete.");
}

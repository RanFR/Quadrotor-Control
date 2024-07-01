#ifndef QUADROTOR_CONTROL_HPP
#define QUADROTOR_CONTROL_HPP

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/MessageInterval.h>

class QuadrotorControl {
private:
    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::ServiceClient arming_client_, set_mode_client_, imu_freq_client_;

    mavros_msgs::State current_state_;

public:
    QuadrotorControl() {};
    ~QuadrotorControl() {};

    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void init(const ros::NodeHandle& nh);
    void run();
};

#endif

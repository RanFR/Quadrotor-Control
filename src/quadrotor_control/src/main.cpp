#include "quadrotor_control/quadrotor_control.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "quadrotor_control");
    ros::NodeHandle nh("~");

    QuadrotorControl quadrotor_control;
    quadrotor_control.init(nh);
    // ros spin has been in the init function

    return 0;
}

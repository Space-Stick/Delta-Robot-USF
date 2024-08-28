#ifndef MY_ROBOT_HW_INTERFACE_H
#define MY_ROBOT_HW_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <chrono>

class MyRobot : public hardware_interface::RobotHW
{
public:
    MyRobot();
    ~MyRobot();
    void init();
    void read();
    void write();
    void tic_get_variable(uint8_t address, uint8_t offset, uint8_t * buffer, uint8_t length);
    void exit_safe_start();

protected:
    ros::NodeHandle nh_;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;

    double cmd[3];
    double pos[3];
    double vel[3];
    double eff[3];

    const char * device = "/dev/i2c-0";
    int fd;
    const int stepsPerRevolution = 16000;
    const double stepsToRadians = 2 * M_PI / stepsPerRevolution;
};

#endif // MY_ROBOT_HW_INTERFACE_H
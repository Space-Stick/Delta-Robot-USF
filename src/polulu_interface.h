#ifndef POLULU_INTERFACE_H
#define POLULU_INTERFACE_H

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

class MyRobot
{
public:
    int32_t cmd[3] = {-264, -264, -264}; //sets default position to -400 z 0x 0y
    int32_t pos[3];
    int32_t velocity[3];
    int8_t home_status[3];
    int32_t threshold_for_position_reached = 20;
    int32_t steps_per_10000seconds = 60000000/10; //maxiumum speed is 160000000, but that is really fast so be careful;
    int32_t offset[3] = {-500, -253, -470}; 
    std::chrono::steady_clock::time_point last_read_time = std::chrono::steady_clock::now(); // Timestamp of the last read

    MyRobot();
    ~MyRobot();
    void init();
    void read();
    void set_home_offset();
    void write();
    void write_max_vel();
    void go_home();
    void get_home_status();
    void set_command(int mot_num, int cmd);
    void tic_get_variable(uint8_t address, uint8_t offset, uint8_t * buffer, uint8_t length);
    void exit_safe_start();
    void setCmdValue(size_t index, double value);
    int fd;
    const char * device = "/dev/i2c-0";
protected:

    const int stepsPerRevolution = 16000;
    const double stepsToRadians = 2 * M_PI / stepsPerRevolution;
};

#endif // POLULU_INTERFACE
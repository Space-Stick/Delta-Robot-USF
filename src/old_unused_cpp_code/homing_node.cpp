#include "ros/ros.h"
#include "my_robot/homing_and_status.h"
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <sstream>


ros::Publisher pub;


int open_i2c_device()
{
  int fd = open("/dev/i2c-0", O_RDWR);
  if (fd == -1)
  {
    perror("/dev/i2c-0");
    return -1;
  }
  return fd;
}

class i2cMotor {
public:
// Sends the "Exit safe start" command.
// Returns 0 on success and -1 on failure.
int tic_exit_safe_start()
{
  uint8_t command[] = { 0x83 };
  struct i2c_msg message = { address, 0, sizeof(command), command };
  struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
  int result = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (result != 1)
  {
    perror("failed to exit safe start");
    return -1;
  }
  return 0;
}
int go_home()
{
  uint8_t command[] = { 0x97, 0x00 };
  struct i2c_msg message = { address, 0, sizeof(command), command };
  struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
  int result = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (result != 1)
  {
    perror("failed to start home");
    return -1;
  }
  return 0;
}
 

int tic_get_home_status(uint8_t * buffer)
{
  uint8_t command[] = { 0xA1, 0x01};
  struct i2c_msg messages[] = {
    { address, 0, sizeof(command), command },
    { address, I2C_M_RD, sizeof(buffer), buffer },
  };
  struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };
  int result = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (result != 2)
  {
    perror("failed to get variables");
    return -1;
  }
  return 0;
}
__u16 address;
int fd;
};
 




void homeCallback(const my_robot::homing_and_status::ConstPtr& msg) {
    if (msg->home) {
        int result;
        int fd = open_i2c_device();
        i2cMotor Mot1;
        i2cMotor Mot2;
        i2cMotor Mot3;
        Mot1.address = 1;
        Mot2.address = 2;
        Mot3.address = 3;
        Mot1.fd = fd;
        Mot2.fd = fd;
        Mot3.fd = fd;

        uint8_t status1;
        uint8_t status2;
        uint8_t status3;
        Mot1.tic_exit_safe_start();
        Mot2.tic_exit_safe_start();
        Mot3.tic_exit_safe_start();
        Mot1.go_home();
        Mot2.go_home();
        Mot3.go_home();

        bool mot1_homed = false;
        bool mot2_homed = false;
        bool mot3_homed = false;

        while (!(mot1_homed && mot2_homed && mot3_homed)) { // does not exit until all 3 motors are homed
          result = Mot1.tic_get_home_status( &status1);
          if (result) {ROS_ERROR("Home Status not retrieved Mot1"); ros::shutdown();}
          result = Mot2.tic_get_home_status( &status2);
          if (result) {ROS_ERROR("Home Status not retrieved Mot2"); ros::shutdown();}
          result = Mot3.tic_get_home_status( &status3);
          if (result) {ROS_ERROR("Home Status not retrieved Mot3"); ros::shutdown();}
          mot1_homed = (status1 & (1 << 4)) == 0;
          mot2_homed = (status2 & (1 << 4)) == 0;
          mot3_homed = (status3 & (1 << 4)) == 0;

          ROS_INFO("Motors homing status: Mot1=%d, Mot2=%d, Mot3=%d", mot1_homed, mot2_homed, mot3_homed);

          ros::Duration(0.5).sleep();
        }
        close(fd);
        my_robot::homing_and_status newMsg;
        newMsg.home = false;
        newMsg.position_uncertain = false;
        pub.publish(newMsg);
        ROS_INFO("Home complete");
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "homing_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/my_robot/home_status", 10, homeCallback);
    pub = nh.advertise<my_robot::homing_and_status>("/my_robot/home_status", 10, true);
    my_robot::homing_and_status newMsg;
    newMsg.position_uncertain = true;
    pub.publish(newMsg);
    ROS_INFO("Home node started");
    ros::spin();

    return 0;
}
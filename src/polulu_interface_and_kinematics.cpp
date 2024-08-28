#include "polulu_interface.h"
#include "my_robot/cmd.h"
#include "my_robot/homing_and_status.h"
#include "my_robot/shutdown.h"
#include "my_robot/position_reached.h"
#include <cmath>
#include <chrono>
#include <iostream>
#include <thread>

MyRobot::MyRobot(){
  
  init();

}

MyRobot::~MyRobot() {
    close(fd);
}

void MyRobot::init(){ 
    fd = open(device, O_RDWR);
}

 
void MyRobot::read(){
  //read the position

for (uint8_t address = 1; address <= 3; ++address) {
        uint8_t buffer[4];
        tic_get_variable(address, 0x22, buffer, sizeof(buffer));
            pos[address - 1] = buffer[0] + 
                               ((uint32_t)buffer[1] << 8) +
                               ((uint32_t)buffer[2] << 16) +
                               ((uint32_t)buffer[3] << 24);

        }
}


void MyRobot::write() {

    exit_safe_start();
    // cmd[0] = 0;
    // cmd[1] = 0;
    // cmd[2] = 0;
for (uint8_t address = 1; address <= 3; ++address) {
        int32_t target = cmd[address - 1];
      
        uint8_t command[] = {
            0xE0,
            static_cast<uint8_t>((target >> 0) & 0xFF),
            static_cast<uint8_t>((target >> 8) & 0xFF),
            static_cast<uint8_t>((target >> 16) & 0xFF),
            static_cast<uint8_t>((target >> 24) & 0xFF),
        };
        struct i2c_msg message = { address, 0, sizeof(command), command };
        struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
        
        int result = ioctl(fd, I2C_RDWR, &ioctl_data);
}

}

void MyRobot::write_max_vel() {

for (uint8_t address = 1; address <= 3; ++address) {
        int32_t target = velocity[address - 1];
        uint8_t command[] = {
            0xE6,
            static_cast<uint8_t>((target >> 0) & 0xFF),
            static_cast<uint8_t>((target >> 8) & 0xFF),
            static_cast<uint8_t>((target >> 16) & 0xFF),
            static_cast<uint8_t>((target >> 24) & 0xFF),
        };
        struct i2c_msg message = { address, 0, sizeof(command), command };
        struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
        
        int result = ioctl(fd, I2C_RDWR, &ioctl_data);
}
}

void MyRobot::exit_safe_start() {

    uint8_t command[] = { 0x83 };
    int result = 0; // Initialize result

    for (uint8_t address = 1; address <= 3; ++address) {
        struct i2c_msg message = { address, 0, sizeof(command), command };
        struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };

        // Attempt to send the command to each address
        int temp_result = ioctl(fd, I2C_RDWR, &ioctl_data);

    }
}

void MyRobot::set_home_offset() {

    exit_safe_start();
for (uint8_t address = 1; address <= 3; ++address) {
        int32_t target = offset[address - 1];
        uint8_t command[] = {
            0xEC,
            static_cast<uint8_t>((target >> 0) & 0xFF),
            static_cast<uint8_t>((target >> 8) & 0xFF),
            static_cast<uint8_t>((target >> 16) & 0xFF),
            static_cast<uint8_t>((target >> 24) & 0xFF),
        };
        struct i2c_msg message = { address, 0, sizeof(command), command };
        struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
        
        int result = ioctl(fd, I2C_RDWR, &ioctl_data);
}
}

void MyRobot::tic_get_variable(uint8_t address, uint8_t offset,
  uint8_t * buffer, uint8_t length){
  uint8_t command[] = { 0xA1, offset };
  struct i2c_msg messages[] = {
    { address, 0, sizeof(command), command },
    { address, I2C_M_RD, length, buffer },
  };
  struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };
  int result = ioctl(fd, I2C_RDWR, &ioctl_data);
}

void MyRobot::setCmdValue(size_t index, double value) {
        if (index < sizeof(cmd)/sizeof(cmd[0])) { // Check index validity
            cmd[index] = value;
  }

}

void MyRobot::go_home()
{
for (uint8_t address = 1; address <= 3; ++address) {
        uint8_t command[] = {
            0x97, 0x00
        };
        struct i2c_msg message = { address, 0, sizeof(command), command };
        struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
        
        int result = ioctl(fd, I2C_RDWR, &ioctl_data);
}
}
void MyRobot::get_home_status() {

for (uint8_t address = 1; address <= 3; ++address) {
        uint8_t buffer[1];
        tic_get_variable(address, 0x01, buffer, sizeof(buffer));
            home_status[address-1] = buffer[0];
        }

}
/////////////////////
// ROBOT KINEMATICS//
/////////////////////

// robot geometry
 
 const float e = 41.5;     // end effector
 const float f = 82.55;     // base
 const float re = 476;
 const float rf = 210;
 
 // trigonometric constants
 const float sqrt3 = sqrt(3.0);
 const float pi = 3.141592653;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1/sqrt3;
 
 // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
 }
 
 // inverse kinematics
 // helper functions, calculates angle theta1 (for YZ-pane)
 int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // shift center to edge
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // non-existing point
     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
     return 0;
 }
 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     return status;
 }

ros::Publisher pub;
ros::Publisher pub1;
MyRobot robot;
int loop_frequency = 120;
float x = 0;
float y = 0;
float z = 0;
float theta1, theta2, theta3;
float theta1_backup, theta2_backup, theta3_backup;
const int stepsPerRevolution = 16000;
int robotCmdPrevious[3];
bool position_uncertain = true;

void homeCallback(const my_robot::homing_and_status::ConstPtr& msg) {
    if (msg->position_uncertain){
      position_uncertain = true;
    }
    if (msg->home) {
      for (int i = 0; i < 3; i++) {
    robot.velocity[i] = 3200000/3;
}
        robot.write_max_vel();

        robot.exit_safe_start();
        robot.go_home();

        bool mot1_homed = false;
        bool mot2_homed = false;
        bool mot3_homed = false;

        while (!(mot1_homed && mot2_homed && mot3_homed)) { // does not exit until all 3 motors are homed
        robot.get_home_status();
          mot1_homed = (robot.home_status[0] & (1 << 4)) == 0;
          mot2_homed = (robot.home_status[1] & (1 << 4)) == 0;
          mot3_homed = (robot.home_status[2] & (1 << 4)) == 0;

          ROS_INFO("Motors homing status: Mot1=%d, Mot2=%d, Mot3=%d", mot1_homed, mot2_homed, mot3_homed);

          ros::Duration(0.5).sleep();
        }
        my_robot::homing_and_status newMsg;
        newMsg.home = false;
        newMsg.position_uncertain = false;
        pub.publish(newMsg);
        position_uncertain = false;
        robot.set_home_offset();
        ROS_INFO("Home complete");
    }
}


void convert_Cmd(const my_robot::cmd::ConstPtr& msg) {
    ROS_INFO("callback triggered");
    static ros::Time last_time = ros::Time::now();
    
    x = msg->x;
    y = msg->y;
    z = msg->z;
    theta1_backup = theta1;
    theta2_backup = theta1;
    theta3_backup = theta1;
    int status = delta_calcInverse(x, y, z, theta1, theta2, theta3);
        if(status == 0){
            ROS_INFO("Kinematics conversion successful: Theta 1: %f Theta 2:%f Theta 3: %f", theta1, theta2, theta3);
        }else{
          ROS_INFO("Kinematics conversion fail: location specified does not exist");
          theta1=theta1_backup;
          theta2=theta2_backup;
          theta3=theta3_backup;
        }
  
  if (round((theta1/360)*stepsPerRevolution) < -3000 || round((theta1/360)*stepsPerRevolution) < -3000 || round((theta1/360)*stepsPerRevolution) < -3000){
    ROS_INFO("sanity check failed, not sending new command");
  }else{
  robot.cmd[0] = round((theta1/360)*stepsPerRevolution);
  robot.cmd[1] = round((theta2/360)*stepsPerRevolution);
  robot.cmd[2] = round((theta3/360)*stepsPerRevolution);
  }
for (int i = 0; i < 3; i++) {
  if (robot.cmd[i]<=robot.offset[i]){
    robot.cmd[i] = robot.offset[i];
  }
}

for (int i = 0; i < 3; i++) {
  robot.velocity[i] = robot.steps_per_10000seconds;
}

}

void shutdown(const my_robot::shutdown::ConstPtr& msg){
  if(msg->shutdown == true){
  ros::shutdown();
  }
}

int main(int argc, char** argv)
{
  bool position_reached = true;
  bool position_reached_last_read = true;
  ros::init(argc, argv, "polulu_interface");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("/my_robot/cmd", 100, convert_Cmd);
  ros::Subscriber sub2 = nh.subscribe("/my_robot/home_status", 100, homeCallback);
  ros::Subscriber sub3 = nh.subscribe("/my_robot/shutdown", 100, shutdown);
  pub = nh.advertise<my_robot::homing_and_status>("/my_robot/home_status", 10, true);
  pub1 = nh.advertise<my_robot::position_reached>("/my_robot/position_reached", 10, true);
  ros::Rate loop_rate(loop_frequency);
  // set starting point to reasonable on startup.
  delta_calcInverse(0.0, 0.0, -400.0, theta1, theta2, theta3);
  robot.cmd[0] = round((theta1/360)*stepsPerRevolution);
  robot.cmd[1] = round((theta2/360)*stepsPerRevolution);
  robot.cmd[2] = round((theta3/360)*stepsPerRevolution);
  for (int i = 0; i < 3; i++) {
  if (robot.cmd[i]<=robot.offset[i]){
    robot.cmd[i] = robot.offset[i];
  }
  }

  while (ros::ok())
  {
    ros::spinOnce();
    if(!position_uncertain){

    robot.read();
    //ROS_INFO("updated %i %i %i", robot.pos[0], robot.pos[1], robot.pos[2]);
    
    if((std::abs(robot.pos[0] - robot.cmd[0]) <= robot.threshold_for_position_reached) && (std::abs(robot.pos[1] - robot.cmd[1]) <= robot.threshold_for_position_reached) && (std::abs(robot.pos[2] - robot.cmd[2]) <= robot.threshold_for_position_reached)){
      position_reached = true;
    }
    else{
      position_reached = false;
    }
    
    
    if (!(position_reached == position_reached_last_read)){
    my_robot::position_reached newMsg;
    newMsg.position_reached = position_reached;
    pub1.publish(newMsg);
    ROS_INFO("Postiton reached message sent %s", (position_reached ? "true" : "false"));
    }
    position_reached_last_read = position_reached;
    robot.exit_safe_start();
    robot.write_max_vel();
    robot.write();
    //ROS_INFO("wrote %i %i %i", robot.cmd[0], robot.cmd[1], robot.cmd[2]);
    }

    loop_rate.sleep();
  }


  return 0;
}

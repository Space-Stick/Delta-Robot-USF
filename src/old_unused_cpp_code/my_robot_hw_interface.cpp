#include "my_robot_hw_interface.h"

MyRobot::MyRobot(){
  
  init();

}

MyRobot::~MyRobot() {
}
void MyRobot::init(){ 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle jointStateHandleA("JointA", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(jointStateHandleA);

   hardware_interface::JointStateHandle jointStateHandleB("JointB", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(jointStateHandleB);
   
   hardware_interface::JointStateHandle jointStateHandleC("JointC", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(jointStateHandleC);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle jointPositionHandleA(jnt_state_interface.getHandle("JointA"), &cmd[0]);
   jnt_pos_interface.registerHandle(jointPositionHandleA);

   hardware_interface::JointHandle jointPositionHandleB(jnt_state_interface.getHandle("JointB"), &cmd[1]);
   jnt_pos_interface.registerHandle(jointPositionHandleB);

   hardware_interface::JointHandle jointPositionHandleC(jnt_state_interface.getHandle("JointC"), &cmd[2]);
   jnt_pos_interface.registerHandle(jointPositionHandleC);

   registerInterface(&jnt_pos_interface);

  }

 
void MyRobot::read(){
  pos[0] = 0.0;
  pos[1] = 0.0;
  pos[2] = 0.0;
  pos[3] = 0.0;
  vel[0] = 0.0;
  vel[1] = 0.0;
  vel[2] = 0.0;
  vel[3] = 0.0;
  eff[0] = 0.0;
  eff[1] = 0.0;
  eff[2] = 0.0;
  eff[3] = 0.0;
//   fd = open(device, O_RDWR);
//   //read the position
// for (uint8_t address = 1; address <= 3; ++address) {
//         uint8_t buffer[4];
//         tic_get_variable(address, 0x22, buffer, sizeof(buffer));
//             pos[address - 1] = (buffer[0] + 
//                                ((uint32_t)buffer[1] << 8) +
//                                ((uint32_t)buffer[2] << 16) +
//                                ((uint32_t)buffer[3] << 24)) * stepsToRadians;

//         }
//         close(fd);
    }

void MyRobot::write() {
  ROS_INFO("data: %f %f %f", cmd[0], cmd[1], cmd[2]);
//     fd = open(device, O_RDWR);
//     exit_safe_start();
// for (uint8_t address = 1; address <= 3; ++address) {
//         int32_t target = cmd[address - 1] / stepsToRadians;
//         uint8_t command[] = {
//             0xE0,
//             static_cast<uint8_t>((target >> 0) & 0xFF),
//             static_cast<uint8_t>((target >> 8) & 0xFF),
//             static_cast<uint8_t>((target >> 16) & 0xFF),
//             static_cast<uint8_t>((target >> 24) & 0xFF),
//         };
//         struct i2c_msg message = { address, 0, sizeof(command), command };
//         struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
        
//         int result = ioctl(fd, I2C_RDWR, &ioctl_data);
// }
//     close(fd);
}

void MyRobot::exit_safe_start() {

    // uint8_t command[] = { 0x83 };
    // int result = 0; // Initialize result

    // for (uint8_t address = 1; address <= 3; ++address) {
    //     struct i2c_msg message = { address, 0, sizeof(command), command };
    //     struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };

    //     // Attempt to send the command to each address
    //     int temp_result = ioctl(fd, I2C_RDWR, &ioctl_data);

    // }
}

void MyRobot::tic_get_variable(uint8_t address, uint8_t offset,
  uint8_t * buffer, uint8_t length){
  // uint8_t command[] = { 0xA1, offset };
  // struct i2c_msg messages[] = {
  //   { address, 0, sizeof(command), command },
  //   { address, I2C_M_RD, length, buffer },
  // };
  // struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };
  // int result = ioctl(fd, I2C_RDWR, &ioctl_data);
}



int main(int argc, char** argv)
{
 
  ros::init(argc, argv, "MyRobot_hardware_interface_node");
  ros::NodeHandle nh;

  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);
  cm.loadController("JointA_PositionController");
  ros::Rate loop_rate(60);

  while (ros::ok())
  {

    robot.read();
    cm.update(ros::Time::now(), ros::Duration(0.0167));
    ROS_INFO("updated");
    robot.write();
    loop_rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}

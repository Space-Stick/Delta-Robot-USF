#include "ros/ros.h"
#include "my_robot/cmd.h"
#include "my_robot/position_reached.h"
#include "my_robot/gpio.h"


bool position_reached_bool;
void position_reached(const my_robot::position_reached::ConstPtr& msg){

    ROS_INFO("position reached callback triggered");
}

void send_cmd(float x,float y, float z){

    ROS_INFO("cmd sending triggered");
    
    
position_reached_bool = false;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_and_place_node");
    ros::NodeHandle nh;
    
    
    ros::Rate loop_rate(100);
    ros::Subscriber sub1 = nh.subscribe("/my_robot/position_reached", 100, position_reached); 
    ros::Publisher pub;
    
    pub = nh.advertise<my_robot::cmd>("/my_robot/cmd", 100);
    

    while (ros::ok()) {
        my_robot::cmd msg;
        msg.x = 0.0;
        msg.y = 300.0;
        msg.z = -400;
        pub.publish(msg);
        loop_rate.sleep();
        start:
        if(true){
            ROS_INFO("looping");
            goto start;
        }
    }
return 0;

}
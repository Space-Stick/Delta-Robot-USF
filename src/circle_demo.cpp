#include "ros/ros.h"
#include <cmath>
#include "my_robot/cmd.h"

ros::Publisher pub;


int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_demo_node");
    ros::NodeHandle nh;
    

    ros::Rate loop_rate(120);


    double center_x = 0.0;
    double center_y = 0.0;
    double radius = 20.0;
    double angular_speed = 2;
    float theta1, theta2, theta3;
    double z = -450;

    pub = nh.advertise<my_robot::cmd>("/my_robot/cmd", 100);

    while (ros::ok()) {
        
        // Get current time
        ros::Time current_time = ros::Time::now();

        // Calculate current angle based on time and angular speed
        double angle = angular_speed * current_time.toSec();

        // Calculate x and y coordinates of the point on the circle
        double x = center_x + radius * cos(angle);
        double y = center_y + radius * sin(angle);
        
        

        ROS_INFO("x: %f y: %f z: %f", x, y, z);
        my_robot::cmd msg;
        msg.x = x;
        msg.y = y;
        msg.z = z;
        pub.publish(msg);
        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }
return 0;

}


#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/delta_controller/command", 10);

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("Mot_1");
    traj.joint_names.push_back("Mot_2");
    traj.joint_names.push_back("Mot_3");

    // Define a point in the trajectory
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {1, 1, -1}; // Specify desired joint positions
    point.time_from_start = ros::Duration(1.0); // Reach the point in 1 second
    traj.points.push_back(point);

    // Send the trajectory
    traj_pub.publish(traj);
    ROS_INFO("trajectory sent");
    ros::spin();
    return 0;
}
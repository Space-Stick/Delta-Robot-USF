#!/usr/bin/env python3

import rospy
from my_robot.msg import homing_and_status

def get_user_choice():
    while not rospy.is_shutdown():
        try:
            user_input = input("Enter 'Y' for Yes or 'N' for No: ").upper()
            if user_input == 'Y':
                return True
            elif user_input == 'N':
                return False
            else:
                print("Invalid input. Please enter 'Y' or 'N'.")
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt received. Exiting.")
            rospy.signal_shutdown("KeyboardInterrupt")

if __name__ == '__main__':
    rospy.init_node("homing_test_publisher")
    rospy.loginfo("homing_test_started")

    pub = rospy.Publisher("/my_robot/home_status", homing_and_status, queue_size=10)

    rate = rospy.Rate(2)
    try:
        while not rospy.is_shutdown():
            msg = homing_and_status()
            msg.home = get_user_choice()
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass  # Handle ROS interrupt exception if needed
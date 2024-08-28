#!/usr/bin/env python3

import rospy
from my_robot.msg import gpio, shutdown
import Jetson.GPIO as GPIO

gripper_pin = 26
enable_pin = 24

GPIO.setmode(GPIO.BOARD)
GPIO.setup(gripper_pin, GPIO.OUT)
GPIO.setup(enable_pin, GPIO.OUT)
GPIO.output(gripper_pin, GPIO.HIGH)
GPIO.output(enable_pin, GPIO.HIGH)

def callback(data):

    if data.gripper_on:
        rospy.loginfo("gripper on")
        GPIO.output(gripper_pin, GPIO.LOW)
    else:
        GPIO.output(gripper_pin, GPIO.HIGH)
        rospy.loginfo("gripper off")

    if data.enable_motors:
        rospy.loginfo("Enable on")
        GPIO.output(enable_pin, GPIO.LOW)
    else:
        GPIO.output(enable_pin, GPIO.HIGH)
        rospy.loginfo("Enable off")


    


def shutdown_callback():
    GPIO.output(gripper_pin, GPIO.HIGH)
    GPIO.output(enable_pin, GPIO.HIGH)
    GPIO.cleanup()

def shutdown_node(msg):
    if msg.shutdown == True:
        rospy.signal_shutdown("recieved shutdown message")



def listener():
    rospy.init_node('gpio_interface', anonymous=True)

    sub = rospy.Subscriber("/my_robot/gpio_status", gpio, callback)
    sub1 = rospy.Subscriber("/my_robot/shutdown", shutdown, shutdown_node)

    # Register shutdown callback
    rospy.on_shutdown(shutdown_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

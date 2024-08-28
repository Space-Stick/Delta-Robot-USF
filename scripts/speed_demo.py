#!/usr/bin/env python

import rospy

from my_robot.msg import cmd, position_reached, gpio, object_description




class PublisherNode:
    def __init__(self):
        rospy.init_node('pick_and_place', anonymous=True)
        self.pub = rospy.Publisher('/my_robot/cmd', cmd, queue_size=10)
        self.pub2 = rospy.Publisher('/my_robot/gpio_status', gpio, queue_size=10)
        self.sub = rospy.Subscriber('/my_robot/position_reached', position_reached, self.position_callback)
        self.sub1 = rospy.Subscriber('/my_robot/object_description', object_description, self.object_description_callback)
        self.rate = rospy.Rate(100)
        self.is_shutdown = False
        self.position_reached = False
        self.x = 0
        self.y = 0
        self.z = 0
        self.object_x = 0
        self.object_y = -300
        self.object_x_true = 0
        self.object_y_true = -300
        self.object_ready = True
        self.object_description = 1
        self.true_object_description = 1
        

    def position_callback(self, msg):
        if msg.position_reached == True:
            rospy.loginfo("position_reached")
            self.position_reached = True
        else:
            self.position_reached = False
        

    def object_description_callback(self, msg):
        preprocessed_x = msg.x
        preprocessed_y = msg.y
        self.object_description = msg.object_description
        self.object_x  = -((preprocessed_x/(146.07/117.0))-76.7)
        self.object_y  = (preprocessed_y/(69.2/56))-390.35
        rospy.loginfo("callback object %f %f",self.object_x, self.object_y)

    def goto_pos(self, x, y, z):
        if (self.x == x and self.y == y and self.z == z) :
            rospy.loginfo('same cmd')
            self.position_reached = True
        while True:
            if self.position_reached:
                break  # Exit the loop if position is reached
            if self.is_shutdown:
                break  
            rospy.loginfo("%f %f %f", x, y, z)
            self.x = x
            self.y = y
            self.z = z
            self.publish_loop()
            rospy.loginfo("executing")
            self.rate.sleep()
        self.position_reached = False  # Reset position_reached for next use
        

    def publish_loop(self):
                msg = cmd()
                msg.x = self.x
                msg.y = self.y
                msg.z = self.z
                self.pub.publish(msg)


    def gripper_on(self):
                msg = gpio()
                msg.gripper_on = True
                msg.enable_motors = True
                node.pub2.publish(msg)
                
    def gripper_off(self):
                msg = gpio()
                msg.gripper_on = False
                msg.enable_motors = True
                node.pub2.publish(msg)
    def shutdown_call(self):
        rospy.loginfo("shutdown")
        rospy.signal_shutdown('shutdown')


                

        
      
greencube_track = 0
      

node = PublisherNode()
rospy.on_shutdown(node.shutdown_call)

while not rospy.is_shutdown():
    
    

    node.goto_pos(0,0,-420)
    node.goto_pos(-120,0,-420)
    rospy.sleep(0.15)
    node.goto_pos(-120,0,-470)
    node.gripper_on()
    rospy.sleep(0.15)
    node.goto_pos(0,0,-420)
    node.goto_pos(120,0,-420)
    rospy.sleep(0.15)
    node.goto_pos(120,0,-470)
    node.gripper_off()
    rospy.sleep(0.15)
    node.goto_pos(120,0,-420)
    node.goto_pos(0,-120,-420)
    rospy.sleep(0.15)
    node.goto_pos(0,-120,-500)
    node.gripper_on()
    rospy.sleep(0.15)
    node.goto_pos(0,-120,-420)
    node.goto_pos(0,0,-420)
    node.goto_pos(0,120,-420)
    rospy.sleep(0.15)
    node.goto_pos(0,120,-500)
    node.gripper_off()
    rospy.sleep(0.15)
    node.goto_pos(0,120,-420)

    node.goto_pos(0,0,-420)
    node.goto_pos(120,0,-420)
    rospy.sleep(0.15)
    node.goto_pos(120,0,-470)
    node.gripper_on()
    rospy.sleep(0.15)
    node.goto_pos(120,0,-420)
    node.goto_pos(0,0,-420)
    node.goto_pos(-120,0,-420)
    rospy.sleep(0.15)
    node.goto_pos(-120,0,-470)
    node.gripper_off()
    rospy.sleep(0.15)
    node.goto_pos(-120,0,-420)
    node.goto_pos(0,120,-420)
    rospy.sleep(0.15)
    node.goto_pos(0,120,-500)
    node.gripper_on()
    rospy.sleep(0.15)
    node.goto_pos(0,120,-420)
    node.goto_pos(0,0,-420)
    node.goto_pos(0,-120,-420)
    rospy.sleep(0.15)
    node.goto_pos(0,-120,-500)
    node.gripper_off()
    rospy.sleep(0.15)
    node.goto_pos(0,-120,-420)

    

    
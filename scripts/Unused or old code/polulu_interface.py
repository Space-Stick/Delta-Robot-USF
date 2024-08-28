#!/usr/bin/env python3

import rospy
from my_robot.msg import stepper_steps
from smbus2 import SMBus, i2c_msg

class TicI2C(object): #defines commands to send over i2c. Not all are used but might be in the future so leave it alone.
  def __init__(self, bus, address):
    self.bus = bus
    self.address = address
 
  # Sends the "Exit safe start" command.
  def exit_safe_start(self):
    command = [0x83]
    write = i2c_msg.write(self.address, command)
    self.bus.i2c_rdwr(write)
 
  # Sets the target position.
  #
  # For more information about what this command does, see the
  # "Set target position" command in the "Command reference" section of the
  # Tic user's guide.
  def set_target_position(self, target):
    command = [0xE0,
      target >> 0 & 0xFF,
      target >> 8 & 0xFF,
      target >> 16 & 0xFF,
      target >> 24 & 0xFF]
    write = i2c_msg.write(self.address, command)
    self.bus.i2c_rdwr(write)
 
  # Gets one or more variables from the Tic.
  def get_variables(self, offset, length):
    write = i2c_msg.write(self.address, [0xA1, offset])
    read = i2c_msg.read(self.address, length)
    self.bus.i2c_rdwr(write, read)
    return list(read)
 
  # Gets the "Current position" variable from the Tic.
  def get_current_position(self):
    b = self.get_variables(0x22, 4)
    position = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24)
    if position >= (1 << 31):
      position -= (1 << 32)
    return position

bus = SMBus(0) #initilize I2c bus 0
motor_1= TicI2C(bus, 1) #initilize addresses for individual tics and thus motors 
motor_2= TicI2C(bus, 2)
motor_3= TicI2C(bus, 3)

def write_position(msg): 
    motor_1.exit_safe_start()
    motor_2.exit_safe_start()
    motor_3.exit_safe_start()
    motor_1.set_target_position(msg.mot_1) #sends pos command to steppers and stepper will begin to move.
    motor_2.set_target_position(msg.mot_2)
    motor_3.set_target_position(msg.mot_3)
    rospy.loginfo("motor positions sent to i2c")
    rospy.loginfo(msg)

if __name__ == '__main__': #ros stuff
    rospy.init_node("polulu_interface")

    sub= rospy.Subscriber("/my_robot/stepper_steps", stepper_steps, callback=write_position)

    rospy.loginfo("Polulu Interface started")

    rospy.spin()
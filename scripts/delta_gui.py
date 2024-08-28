#!/usr/bin/env python3
import rospy
from tkinter import Tk, Label, Entry, Button, messagebox
from my_robot.msg import cmd, homing_and_status, gpio, shutdown





class DeltaGUI:
    def __init__(self, master):

        self.pub = rospy.Publisher("/my_robot/cmd", cmd, queue_size=10)
        self.pub2 = rospy.Publisher("/my_robot/home_status", homing_and_status, queue_size=10)
        self.pub3 = rospy.Publisher("/my_robot/gpio_status", gpio, queue_size=10)
        self.pub4 = rospy.Publisher("/my_robot/shutdown", shutdown, queue_size=10)
        self.sub = rospy.Subscriber("/my_robot/home_status", homing_and_status, self.homeCallback);

        

        self.position_uncertain = True
        self.enable_value = False
        self.gripper_value = False
        
        self.increment = 10  # Default increment value

        self.master = master
        master.title("Delta Gui")

        self.label_X = Label(master, text="X coordinate (mm)")
        self.label_X.grid(row=0, column=0)
        self.label_Y = Label(master, text="Y coordinate (mm)")
        self.label_Y.grid(row=1, column=0)
        self.label_Z = Label(master, text="Z coordinate (mm)")
        self.label_Z.grid(row=2, column=0)

        self.entry_X = Entry(master)
        self.entry_X.grid(row=0, column=1)
        self.entry_X.insert(0, "0")
        self.entry_Y = Entry(master)
        self.entry_Y.grid(row=1, column=1)
        self.entry_Y.insert(0, "0")
        self.entry_Z = Entry(master)
        self.entry_Z.grid(row=2, column=1)
        self.entry_Z.insert(-400, "-400")

        self.publish_button = Button(master, text="Publish coordinates", command=self.publish_values)
        self.publish_button.grid(row=3, column=0, columnspan=2)

        self.homing_button = Button(master, text="Start Homing", command=self.start_homing)
        self.homing_button.grid(row=3, column=2, columnspan=2)

        self.enable_button = Button(master, text="Enable_toggle", command=self.toggle_enable)
        self.enable_button.grid(row=4, column=0, columnspan=2)
        self.enable_button.config(bg="red")

        self.gripper_button = Button(master, text="Gripper_toggle", command=self.toggle_gripper) 
        self.gripper_button.grid(row=4, column=2, columnspan=2)
        self.gripper_button.config(bg="red")

        # Create buttons for incrementing/decrementing X, Y, and Z positions
        self.x_inc_button = Button(master, text="X +", command=lambda: self.update_position('x', self.increment))
        self.x_inc_button.grid(row=6, column=0)
        self.x_dec_button = Button(master, text="X -", command=lambda: self.update_position('x', -self.increment))
        self.x_dec_button.grid(row=6, column=1)

        self.y_inc_button = Button(master, text="Y +", command=lambda: self.update_position('y', self.increment))
        self.y_inc_button.grid(row=6, column=2)
        self.y_dec_button = Button(master, text="Y -", command=lambda: self.update_position('y', -self.increment))
        self.y_dec_button.grid(row=6, column=3)

        self.z_inc_button = Button(master, text="Z +", command=lambda: self.update_position('z', self.increment))
        self.z_inc_button.grid(row=6, column=4)
        self.z_dec_button = Button(master, text="Z -", command=lambda: self.update_position('z', -self.increment))
        self.z_dec_button.grid(row=6, column=5)

    def toggle_enable(self):
        self.enable_value = not self.enable_value
        self.update_button_color(self.enable_button, self.enable_value)
        self.publish_gpio_status()
        if self.enable_value == False:
            msg = homing_and_status()
            msg.position_uncertain = True;
            self.pub2.publish(msg)


    def toggle_gripper(self):
        self.gripper_value = not self.gripper_value
        self.update_button_color(self.gripper_button, self.gripper_value)
        self.publish_gpio_status()

    def update_position(self, axis, value):
        if self.position_uncertain == True:
            messagebox.showerror("Error", "Position uncertain, please rehome Note: When enable is triggered, position becomes uncertain")
        else:
            entry_widget = None
            if axis == 'x':
                entry_widget = self.entry_X
            elif axis == 'y':
                entry_widget = self.entry_Y
            elif axis == 'z':
                entry_widget = self.entry_Z
            else:
                rospy.logwarn("Invalid axis specified.")
                return

            current_value = self.get_float(entry_widget)
            if current_value is not None:
                new_value = current_value + value
                entry_widget.delete(0, 'end')
                entry_widget.insert(0, str(new_value))
            else:
                rospy.logwarn("Invalid input for axis %s.", axis)

            self.publish_values()

    def update_button_color(self, button, state):
        if state:
            button.config(bg="green")  
        else:
            button.config(bg="red")    

    def publish_gpio_status(self):
        msg = gpio()
        msg.enable_motors = self.enable_value
        msg.gripper_on = self.gripper_value
        self.pub3.publish(msg)
        rospy.loginfo("Enable: %s, Gripper: %s", self.enable_value, self.gripper_value)

    def get_float(self, entry_widget): 
        try:
            user_input = float(entry_widget.get())
            return user_input
        except ValueError:
            rospy.logwarn("Invalid input.")
            return None

    def publish_values(self):
        if self.position_uncertain == True:
            messagebox.showerror("Error", "Position uncertain, please rehome Note: When enable is triggered, position becomes uncertain")
        else:
            X = self.get_float(self.entry_X)
            Y = self.get_float(self.entry_Y)
            Z = self.get_float(self.entry_Z)

            if X is not None and Y is not None and Z is not None:
                msg = cmd()
                msg.x = X
                msg.y = Y
                msg.z = Z
                self.pub.publish(msg)
                rospy.loginfo("Values published: X: {}, Y: {}, Z: {}".format(X, Y, Z))

    def start_homing(self):
        
        msg = cmd()
        msg.x = 0
        msg.y = 0
        msg.z = -400
        self.pub.publish(msg)

        msg = homing_and_status()
        msg.home = True
        self.pub2.publish(msg)
        rospy.loginfo("Homing process started")
    
    def on_closing(self):
        self.close_ros_nodes()
        self.master.destroy()
    
    def close_ros_nodes(self):
        # Close ROS publishers and subscribers
        msg = shutdown()
        msg.shutdown = True
        self.pub4.publish(msg)

        self.pub.unregister()
        self.pub2.unregister()
        self.pub3.unregister()
        self.sub.unregister()
        # Shutdown ROS node
        rospy.signal_shutdown("Closing all ROS nodes")

    def homeCallback(self, data):
        self.position_uncertain = data.position_uncertain
        rospy.loginfo(" position uncertain : %s", str(self.position_uncertain))


        

        

if __name__ == '__main__':
    
    
    rospy.init_node("delta_gui")
    rospy.loginfo("delta_gui started") 

   
    root = Tk()
    gui = DeltaGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()
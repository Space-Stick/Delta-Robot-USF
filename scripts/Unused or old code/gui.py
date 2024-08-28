import tkinter as tk
from stepper_control import StepperControl

class GUI:
    def __init__(self, master):
        self.master = master
        self.master.title('Stepper Motor Control')
        self.stepper_control = StepperControl()

        self.motor1_entry = tk.Entry(master)
        self.motor2_entry = tk.Entry(master)
        self.motor3_entry = tk.Entry(master)

        self.motor1_entry.pack()
        self.motor2_entry.pack()
        self.motor3_entry.pack()

        self.set_positions_button = tk.Button(master, text='Set Positions', command=self.set_positions)
        self.set_positions_button.pack()

    """  def set_positions(self):
        motor1_pos = int(self.motor1_entry.get())
        motor2_pos = int(self.motor2_entry.get())
        motor3_pos = int(self.motor3_entry.get())
        self.stepper_control.set_motor_positions(motor1_pos, motor2_pos, motor3_pos) """

if __name__ == '__main__':
    root = tk.Tk()
    gui = GUI(root)
    root.mainloop()


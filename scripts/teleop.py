#!/usr/bin/env python
"""
Joystick Teleop

Uses a usb joystick for teleoperation. Publishes command to /cmd_vel topic.
Designed for and tested on an Xbox 360 controller

ROS Parameters:
- x_threshold = joystick deadzone for x
- y_threshold = joystick deadzone for y
- rate = node update rate
"""
import inputs
import rospy
from geometry_msgs.msg import Twist
from warmup_project.msg import JoystickInput

def rescale(in_min, in_max, out_min, out_max, in_val):
    scale = (float(in_val) - float(in_min)) / (float(in_max) - float(in_min))
    output = (float(out_max) - float(out_min))*scale + float(out_min)
    return output

class JoystickTeleop():
    def __init__(self):
        rospy.init_node("joystick_teleop")

        rate = rospy.get_param('~rate', 1000)
        self.update_rate = rospy.Rate(rate)

        self.x_thresh = rospy.get_param('~x_threshold', 1000)
        self.y_thresh = rospy.get_param('~y_threshold', 6000)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.button_pub = rospy.Publisher("/button_states", JoystickInput, queue_size=10)

        self.pads = inputs.devices.gamepads
        if len(self.pads) == 0:
            raise Exception("Couldn't find any Gamepads!")
        else:
            rospy.loginfo("Found %s", self.pads)
            rospy.loginfo("Starting teleop! \n" + \
                          "Keybindings: \n" + \
                          "- A: Activate \n" + \
                          "- B: E-stop \n" + \
                          "- DPad Up/Down: Change State\n" + \
                          "States: \n" + \
                          "- 0: Teleop \n" + \
                          "- 1: Wall Follow \n" + \
                          "- 2: Person Follow \n" + \
                          "- 3: Avoid Obstacles")

        self.curr_x_input = 0
        self.curr_y_input = 0
        self.curr_dpad_y = 0
        self.curr_btn_south = 0 # a button
        self.curr_btn_east = 0 # b button

    def computeTwistCmd(self, x, y):
        """ Compute twist command based on current x and y joystick values

        y input range: (32768, -32768) (backwards, forwards)
        x input range: (-32768, 32768) (left, right)

        linear velocity output range: (-1, 1) (backwards, forwards)
        angular velocity output range: (1, -1) (left, right)
        """
        lin_vel = rescale(32768, -32768, -1, 1, y)
        ang_vel = rescale(-32768, 32768, 1, -1, x)

        output = Twist()
        output.linear.x = lin_vel
        output.angular.z = ang_vel

        return output

    def computeButtonState(self, dpad_y, btn_south, btn_east):
        """ Create button state message based on raw joystick values

        dpad_y: -1 if pressed up, 1 if pressed down
        btn_south: The 'a' button, 1 if pressed
        btn_east: The 'b' button, 1 if pressed

        Returns a JoystickInput message
        """
        output = JoystickInput()
        if dpad_y == -1:
            output.dpad_up = True
        elif dpad_y == 1:
            output.dpad_down = True
        if btn_south == 1:
            output.a = True
        if btn_east == 1:
            output.b = True
        return output

    def run(self):
        while not rospy.is_shutdown():
            events = inputs.get_gamepad()

            for event in events:
                # print(event.ev_type, event.code, event.state)
                if event.code == 'ABS_Y':
                    self.curr_y_input = event.state
                elif event.code == 'ABS_X':
                    self.curr_x_input = event.state
                elif event.code == 'ABS_HAT0Y':
                    self.curr_dpad_y = event.state
                elif event.code == 'BTN_SOUTH':
                    self.curr_btn_south = event.state
                elif event.code == 'BTN_EAST':
                    self.curr_btn_east = event.state

            new_cmd = self.computeTwistCmd(self.curr_x_input, self.curr_y_input)
            self.twist_pub.publish(new_cmd)

            new_btn_state = self.computeButtonState(self.curr_dpad_y,
                                                    self.curr_btn_south,
                                                    self.curr_btn_east)
            self.button_pub.publish(new_btn_state)

            self.update_rate.sleep()


if __name__ == "__main__":
    jt = JoystickTeleop()
    jt.run()

#!/usr/bin/env python

import inputs
import rospy
from geometry_msgs.msg import Twist

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

        self.pads = inputs.devices.gamepads
        if len(self.pads) == 0:
            raise Exception("Couldn't find any Gamepads!")
        else:
            rospy.loginfo("Found %s", self.pads)

        self.curr_x_input = 0
        self.curr_y_input = 0

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

    def run(self):
        while not rospy.is_shutdown():
            events = inputs.get_gamepad()

            for event in events:
                # print(event.ev_type, event.code, event.state)
                if event.code == 'ABS_Y':
                    self.curr_y_input = event.state
                elif event.code == 'ABS_X':
                    self.curr_x_input = event.state

            new_cmd = self.computeTwistCmd(self.curr_x_input, self.curr_y_input)
            self.twist_pub.publish(new_cmd)

            self.update_rate.sleep()


if __name__ == "__main__":
    jt = JoystickTeleop()
    jt.run()

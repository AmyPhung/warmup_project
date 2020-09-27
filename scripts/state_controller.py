#!/usr/bin/env python

"""
in launch file, remap all topics to differently named

STATES = {
    1: "/teleop/cmd_vel"
    2: "/wall_follower/cmd_vel"

"""
import rospy
from geometry_msgs.msg import Twist
from warmup_project.msg import JoystickInput

class StateController():
    def __init__(self):
        rospy.init_node("state_controller")
        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)

        self.button_pub = rospy.Subscriber("/button_states",
            JoystickInput, self.buttonCB)
        self.teleop_sub = rospy.Subscriber("/teleop/cmd_vel",
            Twist, self.teleopCB)
        self.wall_follow_sub = rospy.Subscriber("/wall_follower/cmd_vel",
            Twist, self.wallFollowCB)
        self.person_follow_sub = rospy.Subscriber("/person_follower/cmd_vel",
            Twist, self.personFollowCB)
        self.obstacle_avoidance_sub = rospy.Subscriber("/obstacle_avoidance/cmd_vel",
            Twist, self.obstacleAvoidanceCB)

        self.button_msg = JoystickInput()
        self.teleop_msg = Twist()
        self.wall_follow_msg = Twist()
        self.person_follow_msg = Twist()
        self.obstacle_avoidance_msg = Twist()

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.activated = False
        self.curr_state = 0

    def teleopCB(self, msg):
        self.teleop_msg = msg

    def wallFollowCB(self, msg):
        self.wall_follow_msg = msg

    def personFollowCB(self, msg):
        self.person_follow_msg = msg

    def obstacleAvoidanceCB(self, msg):
        self.obstacle_avoidance_msg = msg

    def buttonCB(self, msg):
        self.button_msg = msg


    def run(self):
        while not rospy.is_shutdown():
            # Check activation state
            if self.button_msg.a:
                self.activated = True
                rospy.loginfo("Activated")
            if self.button_msg.b:
                self.activated = False
                rospy.loginfo("E-stopped")

            if self.button_msg.dpad_up:
                self.curr_state += 1
                rospy.loginfo("Switching to state " + str(self.curr_state))
            if self.button_msg.dpad_down:
                self.curr_state += -1
                rospy.loginfo("Switching to state " + str(self.curr_state))

            if self.activated:
                if self.curr_state == 0:
                    self.twist_pub.publish(self.teleop_msg)
                elif self.curr_state == 1:
                    self.twist_pub.publish(self.wall_follow_msg)
                elif self.curr_state == 2:
                    self.twist_pub.publish(self.person_follow_msg)
                elif self.curr_state == 3:
                    self.twist_pub.publish(self.obstacle_avoidance_msg)
                else:
                    # Publish stopped message for safety if state is invalid
                    self.twist_pub.publish(Twist())
            else:
                # Publish stopped message for safety if e-stopped
                self.twist_pub.publish(Twist())

            self.update_rate.sleep()


if __name__=="__main__":
    state_controller = StateController()
    state_controller.run()

#!/usr/bin/env python

"""
Obstacle avoidance using the dynamic window approach

Based on: https://github.com/goktug97/DynamicWindowApproach
"""

# ROS Imports
import rospy
import ros_numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler, \
    quaternion_multiply, quaternion_conjugate, unit_vector
from sensor_msgs.msg import PointCloud2, LaserScan
from warmup_project.cfg import ObstacleAvoidanceConfig
from dynamic_reconfigure.server import Server

# Python
import sys
import math
import numpy as np
import laser_geometry.laser_geometry as lg
import matplotlib.pyplot as plt

def computeAngle(vec1, vec2):
    unit_vec1 = vec1 / np.linalg.norm(vec1)
    unit_vec2 = vec2 / np.linalg.norm(vec2)
    dot_product = np.dot(unit_vec1, unit_vec2)
    angle = np.arccos(dot_product)
    sign = np.sign(np.cross(unit_vec1, unit_vec2))
    return sign * angle

def quaternion_mult(q,r):
    return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]

def point_rotation_by_quaternion(point,q):
    r = [0]+point
    q_conj = [q[0],-1*q[1],-1*q[2],-1*q[3]]
    return quaternion_mult(quaternion_mult(q,r),q_conj)[1:]



# Datatypes --------------------------------------------------------
class Rect():
    def __init__(self):
        self.xmin = 0.0
        self.ymin = 0.0
        self.xmax = 0.0
        self.ymax = 0.0

class Config():
    def __init__(self):
        self.maxSpeed = 0.0
        self.minSpeed = 0.0
        self.maxYawrate = 0.0
        self.maxAccel = 0.0
        self.maxdYawrate = 0.0
        self.velocityResolution = 0.0
        self.yawrateResolution = 0.0
        self.dt = 0.0
        self.predictTime = 0.0
        self.heading = 0.0
        self.clearance = 0.0
        self.velocity = 0.0
        self.base = Rect()

class DynamicWindow():
    def __init__(self):
        self.nPossibleV = 0
        self.possibleV = []
        self.nPossibleW = 0
        self.possibleW = []

# Path planner ------------------------------------------------------
class PathPlanner():
    def __init__(self, config):
        self.config = config

    def calculateVelocityCost(self, twist, config):
        return config.maxSpeed - twist.linear.x

    def calculateHeadingCost(self, pose, goal):
        dx = goal.pose.position.x - pose.position.x
        dy = goal.pose.position.y - pose.position.y
        angleError = math.atan2(dy, dx)
        # angleError = computeAngle([1,0], [dx, dy])
        # print(angleError)
        eul = euler_from_quaternion([pose.orientation.x,
                                     pose.orientation.y,
                                     pose.orientation.z,
                                     pose.orientation.w])

        angleCost = angleError - eul[2];
        # print(angleCost)
        return abs(math.atan2(math.sin(angleCost), math.cos(angleCost)));

    def calculateClearanceCost(self, pose, twist, pcl, config):
        time = 0
        minr = sys.maxsize

        # print(pcl[0])

        while (time < config.predictTime):
            # pose = self.projectMotion(pose, twist, config.dt)

            # print(pcl)
            # for i in range(len(pcl.))
            time += config.dt;
        return 1.0 / minr;

    def createDynamicWindow(self, twist, config):
        dw = DynamicWindow()

        minV = max(config.minSpeed, twist.linear.x - config.maxAccel * config.dt)
        maxV = min(config.maxSpeed, twist.linear.x + config.maxAccel * config.dt)
        minW = max(-config.maxYawrate, twist.angular.z - config.maxdYawrate * config.dt);
        maxW = min(config.maxYawrate, twist.angular.z + config.maxdYawrate * config.dt)

        dw.nPossibleV = int((maxV - minV) / config.velocityResolution)
        dw.nPossibleW = int((maxW - minW) / config.yawrateResolution)

        # Tune config until these are a reasonable number
        # print(dw.nPossibleV)
        # print(dw.nPossibleW)

        for i in range(dw.nPossibleV):
            dw.possibleV.append(minV + i * config.velocityResolution)
        for i in range(dw.nPossibleW):
            dw.possibleW.append(minW + i * config.yawrateResolution)
        return dw

    def projectMotion(self, pose, twist, dt):
        """ Predict where robot will be in the next timestep in global frame
        Args:
            pose (geometry_msgs/Pose): Current robot pose in /odom frame
            twist (geometry_msgs/Twist): Current twist from /odom frame
        """
        new_pose = Pose()
        eul = euler_from_quaternion([pose.orientation.x,
                                     pose.orientation.y,
                                     pose.orientation.z,
                                     pose.orientation.w])
        yaw = eul[2] + twist.angular.z * dt;
        quat = quaternion_from_euler(0,0,yaw)
        new_pose.orientation.x = quat[0]
        new_pose.orientation.y = quat[1]
        new_pose.orientation.z = quat[2]
        new_pose.orientation.w = quat[3]

        new_pose.position.x = pose.position.x + twist.linear.x * math.cos(yaw) * dt;
        new_pose.position.y = pose.position.y + twist.linear.y * math.sin(yaw) * dt;
        return new_pose;

    def computeTwistCommand(self, pose, twist, goal, pcl, config):
        """ Compute best twist command with the dynamic window implementation

        Args:
            pose (geometry_msgs/Pose): Current robot pose in /odom frame
            twist (geometry_msgs/Twist): Current twist from /odom frame
            goal (geometry_msgs/Pose): Goal pose in /odom frame
            pcl (sensor_msgs/PointCloud2): PCL generated from laser in robot frame
            config (Config): Settings to use for computation
        """

        dw = self.createDynamicWindow(twist, config)
        pTwist = Twist()
        pPose = pose
        bestTwist = Twist()
        cost = 0
        total_cost = sys.maxsize

        # For debugging
        pPose_array_ = PoseArray()
        pPose_array_.header = goal.header
        best_pose_ = pose

        for i in range(dw.nPossibleV):
            for j in range(dw.nPossibleW):
                pTwist.linear.x = dw.possibleV[i]
                pTwist.angular.z = dw.possibleW[j]
                pPose = self.projectMotion(pPose, pTwist, config.dt)

                cost = config.heading * self.calculateHeadingCost(pPose, goal)
                    #config.velocity * self.calculateVelocityCost(pTwist, config) +

                    # config.clearance * self.calculateClearanceCost(pose, pTwist,
                    #                                           pcl, config)

                # For debugging
                pPose_array_.poses.append(pPose)

                if (cost < total_cost):
                    total_cost = cost
                    bestTwist = pTwist

                    # For debugging
                    best_pose_ = pPose
        print(total_cost)
        return bestTwist, pPose_array_, best_pose_



class ObstacleAvoidance():
    def __init__(self):
        self.ready = False
        rospy.init_node("obstacle_avoidance")
        rospy.loginfo("Starting obstacle avoidance node...")

        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)
        self.visualize = rospy.get_param('~visualize', True)

        srv = Server(ObstacleAvoidanceConfig, self.paramCB)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCB)
        self.odom_msg = Odometry()
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCB)
        self.goal_msg = PoseStamped()
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laserCB)
        self.laser_msg = LaserScan()

        if self.visualize:
            self.projections_pub = rospy.Publisher("/projections", PoseArray, queue_size=10)
            self.best_projection_pub = rospy.Publisher("/best_projection", Pose, queue_size=10)

        robot_clearance = Rect()
        # Assume robot is a 40x40cm box with the center at 0,0
        robot_clearance.xmin = rospy.get_param('~xmin', 0.2)
        robot_clearance.ymin = rospy.get_param('~ymin', 0.2)
        robot_clearance.xmax = rospy.get_param('~xmax', 0.2)
        robot_clearance.ymax = rospy.get_param('~ymax', 0.2)

        self.config = Config()
        self.config.maxSpeed = rospy.get_param('~maxSpeed', 0.50)
        self.config.minSpeed = rospy.get_param('~minSpeed', -0.5)
        self.config.maxYawrate = rospy.get_param('~maxYawrate', 1.00)
        self.config.maxAccel = rospy.get_param('~maxAccel', 1.0)
        self.config.maxdYawrate = rospy.get_param('~maxdYawrate', 1.00)
        self.config.velocityResolution = rospy.get_param('~velocityResolution', 0.00505)
        self.config.yawrateResolution = rospy.get_param('~yawrateResolution', 0.00505)
        self.config.dt = rospy.get_param('~dt', 0.1)
        self.config.predictTime = rospy.get_param('~predictTime', 0.1)
        self.config.heading = rospy.get_param('~heading', 0.15)
        self.config.clearance = rospy.get_param('~clearance', 1.00)
        self.config.velocity = rospy.get_param('~velocity', 1.00)
        self.config.base = robot_clearance

        # Path planner is used to compute next command
        self.path_planner = PathPlanner(self.config)

        # Used to create pointcloud from laser data
        self.lp = lg.LaserProjection()

        rospy.loginfo("Obstacle avoidance node setup complete!")
        self.ready = True

    def paramCB(self, config, level):
        if not self.ready:
            return config

        self.config.maxSpeed = config.maxSpeed
        self.config.minSpeed = config.minSpeed
        self.config.maxYawrate = config.maxYawrate
        self.config.maxAccel = config.maxAccel
        self.config.maxdYawrate = config.maxdYawrate
        self.config.velocityResolution = config.velocityResolution
        self.config.yawrateResolution = config.yawrateResolution
        self.config.dt = config.dt
        self.config.predictTime = config.predictTime
        self.config.heading = config.heading
        self.config.clearance = config.clearance
        self.config.velocity = config.velocity

        # Update path planner
        self.path_planner.config = self.config
        return config

    def odomCB(self, msg):
        self.odom_msg = msg

    def goalCB(self, msg):
        self.goal_msg = msg

    def laserCB(self, msg):
        self.laser_msg = msg

    def run(self):
        while not rospy.is_shutdown():
            # self.twist_pub.publish(twist_cmd)
            # print(self.odom_msg)
            # print(self.config.dt)
            pcl_msg = self.lp.projectLaser(self.laser_msg)

            # Need to convert odom linear twist message to odom frame
            # (defaults to robot frame)
            quat_vec = [self.odom_msg.pose.pose.orientation.w,
                    self.odom_msg.pose.pose.orientation.x,
                    self.odom_msg.pose.pose.orientation.y,
                    self.odom_msg.pose.pose.orientation.z]
            twist_vec = [self.odom_msg.twist.twist.linear.x,
                     self.odom_msg.twist.twist.linear.y,
                     self.odom_msg.twist.twist.linear.z]
            tf_twist_linear = point_rotation_by_quaternion(twist_vec, quat_vec)
            # print(point_rotation_by_quaternion([1, 0, 0],[0.7071203316249954, 0.0, 0.7071203316249954, 0.0])

            twist_msg = Twist()
            twist_msg.linear.x = tf_twist_linear[0]
            twist_msg.linear.y = tf_twist_linear[1]
            twist_msg.linear.z = tf_twist_linear[2]

            # Assume Robot and odom z frames match
            twist_msg.angular.z = self.odom_msg.twist.twist.angular.z

            print(twist_msg)

            # # print(pcl_msg)
            # twist_cmd, pPose_array, best_pose = self.path_planner.computeTwistCommand(
            #     self.odom_msg.pose.pose, twist_msg,
            #     self.goal_msg, pcl_msg, self.config)
            #
            # if self.visualize:
            #     self.projections_pub.publish(pPose_array)# = rospy.Publisher("/projections", PoseArray, queue_size=10)
            #     self.best_projection_pub.publish(best_pose)# = rospy.Publisher("/best_projection", Pose, queue_size=10)
            # # self.projection_pub.publish(pPose)
            # # self.twist_pub.publish(twist_cmd)
            self.update_rate.sleep()

if __name__ == "__main__":

    obstacle_avoidance = ObstacleAvoidance()
    obstacle_avoidance.run()

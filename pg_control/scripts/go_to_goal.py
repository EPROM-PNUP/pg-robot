#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from pg_msgs.msg import GoToGoalAction
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt

class GoToGoal:
	def __init__(self):
		rospy.loginfo("Initializing node ...")
		rospy.init_node("go_to_goal")

		self._pose = Pose2D()
		self._goal_pose = Pose2D()
		
		self._distance_from_goal = 0.0
		self._tolerance_distance = 0.2

		self._VELOCITY = 6.0
		self._linear_x_vel = 0.0
		self._linear_y_vel = 0.0
		self._omega = 0.0

		self._command_velocity_msg = Twist()

		rospy.loginfo("Subscribing to topics ...")
		rospy.Subscriber("odom", Odometry, self._pose_callback)

		rospy.loginfo("Setting up publisher ...")
		self._command_velocity_pub = rospy.Publisher(
			"cmd_vel",
			Twist,
			queue_size=1)

		self.server = actionlib.SimpleActionServer(
			"go_to_goal",
			GoToGoalAction,
			self.execute,
			False)

		self.server.start()
	
	def _pose_callback(self, message):
		self._pose.x = message.pose.pose.position.x
		self._pose.y = message.pose.pose.position.y
		
		orientation_quat = message.pose.pose.orientation
		orientation_list =
		[
			orientation_quat.x,
			orientation_quat.y,
			orientation_quat.z,
			orientation_quat.w
		]

		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)

		self._pose.theta = yaw

	def _euclidian_distance(self):
		self._distance_from_goal = sqrt(
			pow((_goal_pose.x - self._pose.x), 2) +
			pow((_goal_pose.y - self._pose.y), 2)
			)

	def _angular_velocity(self):
		self._omega = self._goal_pose.theta - self._pose.theta
	
	def _linear_velocity(self):
		self._x_linear_vel = (
			(self._VELOCITY * (self._goal_pose.x - self._pose.x)) / 
			self._distance_from_goal
			)

		self._y_linear_vel = (
			(self._VELOCITY * (self._goal_pose.y - self._pose.y)) / 
			self._distance_from_goal
			)

	def execute(self):
		rate = rospy.Rate(10)

		while self._distance_from_goal > self._tolerance_distance:
			self._euclidian_distance()
			self._angular_velocity()
			self._linear_velocity()

			self._command_velocity_msg.linear.x = self._x_linear_vel
			self._command_velocity_msg.linear.y = self._y_linear_vel
			self._commang_velocity_msg.linear.z = 0.0

			self._command_velocity_msg.angular.x = 0.0
			self._command_velocity_msg.angular.y = 0.0
			self._command_velocity_msg.angular.z = self._omega

			self._command_velocity_pub.publish(self._command_velocity_msg)

			self.rate.sleep()
		
		self.server.set_succeeded()

		self._command_velocity_msg.linear.x = 0.0
		self._command_velocity_msg.linear.y = 0.0
		self._commang_velocity_msg.linear.z = 0.0

		self._command_velocity_msg.angular.x = 0.0
		self._command_velocity_msg.angular.y = 0.0
		self._command_velocity_msg.angular.z = 0.0

		self._command_velocity_pub.publish(self._command_velocity_msg)

def main():
	go_to_goal = GoToGoal()
	rospy.spin()

	del go_to_goal

if (__name__ == "__main__"):
	main()

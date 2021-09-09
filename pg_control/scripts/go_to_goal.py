#!/usr/bin/env python3

# Copyright (c) 2021, EPROM PNUP
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Wahyu Mahardika

import rospy
import actionlib

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt

from pg_msgs.msg import GoToGoalAction
from pg_msgs.msg import GoToGoalGoal
from pg_msgs.msg import GoToGoalFeedback
from pg_msgs.msg import GoToGoalResult

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
		
		rospy.loginfo("Starting action servers")
		self._server = actionlib.SimpleActionServer(
			"go_to_goal",
			GoToGoalAction,
			self.execute,
			False)

		self._feedback = GoToGoalFeedback()
		self._result = GoToGoalResult()

		self._server.start()
	
	def _pose_callback(self, message):
		self._pose.x = message.pose.pose.position.x
		self._pose.y = message.pose.pose.position.y
		
		orientation_quat = message.pose.pose.orientation
		orientation_list = [
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

	def execute(self, goal):
		rate = rospy.Rate(10)

		success = True

		self._goal_pose = goal

		while self._distance_from_goal > self._tolerance_distance:
			if self._server.is_preemt_requested():
				rospy.loginfo("go_to_goal: Preempted")
				self._server.set_preempted()
				success = False
				break

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

			self._feedback.x = self._pose.x
			self._feedback.y = self._pose.y
			self._feedback.theta = self._pose.theta

			self._server.publish_feedback(self._feedback)

			self.rate.sleep()

		if success:
			self._result.x = self._pose.x
			self._result.y = self._pose.y
			self._result.theta = self._pose.theta

			rospy.loginfo("go_to_goal: Succeeded")
		
			self._server.set_succeeded(self._result)

		self._command_velocity_msg.linear.x = 0.0
		self._command_velocity_msg.linear.y = 0.0
		self._command_velocity_msg.linear.z = 0.0

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

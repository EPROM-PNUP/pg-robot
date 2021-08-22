#!/usr/bin/env python3

# Copyright (c) 2021 EPROM PNUP
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom
# the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# Author: Wahyu Mahardika

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from pg_msgs.msg import WheelVelocityCommand

# BASE CONTROLLER.
# Receive body twist command and transform it
# into angular velocities for each wheels.
class BaseController:

	def __init__(self):
		rospy.loginfo("Initializing node ...")
		rospy.init_node('base_controller')

		self._last_received = rospy.get_time()
		self._wheel_radius = rospy.get_param('wheel_radius')
		self._timeout = rospy.get_param('base_controller/timeout')
		self._rate = rospy.get_param('base_controller/rate')

		self._wheel_velocity = [0.0, 0.0, 0.0]
		self._body_twist = [0.0, 0.0, 0.0]

		self._wheel_velocity_msg = WheelVelocityCommand() 
		self._wheel_velocity_msg.data = [0.0, 0.0, 0.0]

		rospy.loginfo("Subscribing to topics ...")
		rospy.Subscriber('cmd_vel', Twist, self._velocity_callback)

		rospy.loginfo("Setting up publishers ...")
		self._wheel_velocity_pub = rospy.Publisher(
			'wheel_refrence_velocity', 
			WheelVelocityCommand, 
			queue_size=1)

	# Velocity callback function
	def _velocity_callback(self, message):
		self._last_received = rospy.get_time()

		self._body_twist[0] = message.linear.x
		self._body_twist[1] = message.linear.y
		self._body_twist[2] = message.angular.z
	
	def _update(self):
		# Chassis to wheel velocity transformation matrix
		H = [
			[np.sin(np.pi/3), 1/2, 0.2],
			[-np.sin(np.pi/3), 1/2, 0.2],
			[0, -1, 0.2]
		]

		# Divide each elements of H with wheel radius
		for i in range(3):
			for j in range(3):
				H[i][j] /= self._wheel_radius

		self._wheel_velocity = [0.0, 0.0, 0.0]

		# Calculate each wheel velocities
		for i in range(len(self._wheel_velocity)):
			for j in range(len(self._body_twist)):
				self._wheel_velocity[i] += H[i][j] * self._body_twist[j]

		# Set wheel velocity message for publish
		for i in range(len(self._wheel_velocity_msg.data)):
			self._wheel_velocity_msg.data[i] = self._wheel_velocity[i]

		rospy.logdebug(self._wheel_velocity)

	def run(self):
		rospy.loginfo("Successfully started base_controller node")
		rate = rospy.Rate(self._rate)

		while not rospy.is_shutdown():
			delay = rospy.get_time() - self._last_received

			self._update()

			# If not received body twist command for 2 seconds
			# stop the robot, else publish wheel velocities.
			if (delay < self._timeout):
				self._wheel_velocity_pub.publish(self._wheel_velocity_msg)
			else:
				self._wheel_velocity_msg.data = [0, 0, 0]
				self._wheel_velocity_pub.publish(self._wheel_velocity_msg)

			rate.sleep()

def main():
	base_controller = BaseController()
	base_controller.run()
	del base_controller

if (__name__ == '__main__'):
	main()


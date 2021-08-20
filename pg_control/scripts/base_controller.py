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
from pg_msgs.msg import MotorCommand

# Driver class
class MotorDriver:
	
	# Constructor
	def __init__(self):
		rospy.loginfo("Initializing node")
		rospy.init_node('base_controller')

		self._last_received = rospy.get_time()
		self._wheel_radius = rospy.get_param('wheel_radius')
		self._timeout = rospy.get_param('/base_controller/timeout')
		self._rate = rospy.get_param('/base_controller/rate')

		self._wheel_velocity = [0.0, 0.0, 0.0]
		self._body_twist = [0.0, 0.0, 0.0]

		self._wheel_velocity_msg = MotorCommand() 
		self._wheel_velocity_msg.data = [0.0, 0.0, 0.0]

		rospy.Subscriber('cmd_vel', Twist, self._velocity_callback)

		self.motor_speed_pub = rospy.Publisher(
			'wheel_refrence_velocity', 
			MotorCommand, 
			queue_size=10)

	# Velocity callback function
	def _velocity_callback(self, message):
		self._last_received = rospy.get_time()

		self._body_twist[0] = message.linear.x
		self._body_twist[1] = message.linear.y
		self._body_twist[2] = message.angular.z

		H = [
			[np.sin(np.pi/3), 1/2, 0.2],
			[-np.sin(np.pi/3), 1/2, 0.2],
			[0, -1, 0.2]
		]

		for (i in H):
			for (j in i):
				j = j / self._wheel_radius

		for (i in range(len(self._wheel_velocity))):
			for (j in range(len(self._body_twist))):
				self._wheel_velocity[i] += H[i][j] * self._body_twist[j]

		for (i in range(len(self._wheel_velocity_msg.data))):
			self._wheel_velocity_msg.data[i] = self._wheel_velocity[i]

		rospy.loginfo(self._wheel_velocity)

	# Ros run function
	def run(self):
		rospy.loginfo("Running /base_controller node")
		rate = rospy.Rate(self._rate)

		while not rospy.is_shutdown():
			delay = rospy.get_time() - self._last_received
			if (delay < self._timeout):
				self.motor_speed_pub.publish(self._motors_speed)
			else:
				self._motors_speed.data = [0, 0, 0]
				self.motor_speed_pub.publish(self._motors_speed)
			rate.sleep()

def main():
	md = MotorDriver()
	md.run()
	del md

if (__name__ == '__main__'):
	main()


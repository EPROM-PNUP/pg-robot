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

import numpy as np
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

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

		self._wheel_1_velocity_msg = Float64() 
		self._wheel_1_velocity_msg.data = 0.0

		self._wheel_2_velocity_msg = Float64() 
		self._wheel_2_velocity_msg.data = 0.0

		self._wheel_3_velocity_msg = Float64() 
		self._wheel_3_velocity_msg.data = 0.0

		rospy.loginfo("Subscribing to topics ...")
		rospy.Subscriber('cmd_vel', Twist, self._velocity_callback)

		rospy.loginfo("Setting up publishers ...")
		self._wheel_1_velocity_pub = rospy.Publisher(
			'wheel_1/reference_velocity', 
			Float64, 
			queue_size=1)

		self._wheel_2_velocity_pub = rospy.Publisher(
			'wheel_2/reference_velocity', 
			Float64, 
			queue_size=1)

		self._wheel_3_velocity_pub = rospy.Publisher(
			'wheel_3/reference_velocity', 
			Float64, 
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
		self._wheel_1_velocity_msg.data = self._wheel_velocity[0]
		self._wheel_2_velocity_msg.data = self._wheel_velocity[1]
		self._wheel_3_velocity_msg.data = self._wheel_velocity[2]

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
				self._wheel_1_velocity_pub.publish(self._wheel_1_velocity_msg)
				self._wheel_2_velocity_pub.publish(self._wheel_2_velocity_msg)
				self._wheel_3_velocity_pub.publish(self._wheel_3_velocity_msg)
			else:
				self._wheel_1_velocity_msg.data = 0.0
				self._wheel_2_velocity_msg.data = 0.0
				self._wheel_3_velocity_msg.data = 0.0

				self._wheel_1_velocity_pub.publish(self._wheel_1_velocity_msg)
				self._wheel_2_velocity_pub.publish(self._wheel_2_velocity_msg)
				self._wheel_3_velocity_pub.publish(self._wheel_3_velocity_msg)

			rate.sleep()

def main():
	base_controller = BaseController()
	base_controller.run()
	del base_controller

if (__name__ == '__main__'):
	main()


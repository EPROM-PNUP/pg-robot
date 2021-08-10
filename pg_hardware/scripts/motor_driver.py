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

# IMPORT HEADERS
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

# Driver class
class MotorDriver:
	
	# Constructor
	def __init__(self):
		rospy.loginfo("Initializing node")
		rospy.init_node('motor_driver')

		self._last_received = rospy.get_time()
		self._timeout = rospy.get_param('~timeout', 2)
		self._rate = rospy.get_param('~rate', 10)
		self._max_speed = rospy.get_param('~max_speed', 39.0)

		self._motor_1_speed = 0
		self._motor_2_speed = 0
		self._motor_3_speed = 0

		self._motors_speed = Int16MultiArray() 

		rospy.Subscriber('cmd_vel', Twist, self._velocity_callback)

		self.motor_speed_pub = rospy.Publisher('motor_pwm', Int16MultiArray, queue_size=10)

	# Velocity callback function
	def _velocity_callback(self, message):
		self._last_received = rospy.get_time()

		linear_x = message.linear.x
		linear_y = message.linear.y
		angular_z = message.angular.z

		self._motor_1_speed = (17.320508075688*linear_x)+(10*linear_y)+(4*angular_z)
		self._motor_2_speed = (-17.320508075688*linear_x)+(10*linear_y)+(4*angular_z)
		self._motor_3_speed = (0.0*linear_x)+(-20*linear_y)+(4*angular_z)

		self._motor_1_speed = (self._motor_1_speed/self._max_speed) * 255 
		self._motor_2_speed = (self._motor_2_speed/self._max_speed) * 255
		self._motor_3_speed = (self._motor_3_speed/self._max_speed) * 255

		self._motor_1_speed = int(self._motor_1_speed)
		self._motor_2_speed = int(self._motor_2_speed)
		self._motor_3_speed = int(self._motor_3_speed)

		self._motors_speed.data = [self._motor_1_speed, self._motor_2_speed, self._motor_3_speed]

		rospy.loginfo([self._motor_1_speed, self._motor_2_speed, self._motor_3_speed])

	# Ros run function
	def run(self):
		rospy.loginfo("Running /motor_driver node")
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



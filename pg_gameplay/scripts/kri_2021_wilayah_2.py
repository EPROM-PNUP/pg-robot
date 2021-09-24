#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from pg_msgs.srv import KickBall

on_ball = False

cmd_vel_pub = rospy.Publisher(
        "cmd_vel",
        Twist,
        queue_size=1)

vel_rate = rospy.Rate(50)

vel_msg = Twist()

def move(cmd_vel, duration):
    begin_time = rospy.get_time()

    while True:
        current_time = rospy.get_time()
        
        if (current_time - begin_time < duration.to_sec()):
            cmd_vel_pub.publish(cmd_vel)

            vel_rate.sleep()
        else:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0

            break

def receive_ball():
    while not on_ball:
        rospy.loginfo("Waiting for ball ...")

def pass_ball():
    rospy.wait_for_service("kick")
    try:
        kick_ball = rospy.ServiceProxy("kick", KickBall)
        res = kick_ball(True)
    except rospy.ServiceException as e:
        rospy.logerror(e)

def mission_1():
    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 5.0
    vel_msg.linear.z = 0.0

    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    move(vel_msg, rospy.Duration(1.5))

    receive_ball()

    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0

    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 4.0

    move(vel_msg, rospy.Duration(1.5))

    pass_ball()

    # move()
    # receive_ball()
    # pass_ball()

if __name__ == '__main__':
    try:
        rospy.init_node("kri_2021_node")
        
        while True:
            print("--- KRI 2021 Wilayah II ---")
            print("Misi: 1, 2, 3.")

            mission = int(input("Pilih Misi: "))

            if mission == 1:
                mission_1()
            elif mission == 2:
                mission_2()

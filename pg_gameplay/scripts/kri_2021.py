#!/usr/bin/env python3

import rospy
import actionlib

from pg_msgs.msg import GoToGoalAction
from pg_msgs.msg import GoToGoalGoal

def go_to_goal_client():
	rospy.loginfo("Starting client ...")
	client = actionlib.SimpleActionClient(
		'go_to_goal', 
		GoToGoalAction
		)
	
	rospy.loginfo("Waiting for server ...")
	client.wait_for_server()

	rospy.loginfo("Sending goal ...")
	goal = GoToGoalGoal()

	goal.goal_pose.x = 1.0
	goal.goal_pose.y = 1.0
	goal.goal_pose.theta = 0.0

	client.send_goal(goal)

	try:
		rospy.loginfo("Waiting for result ...")
		client.wait_for_result()
	except rospy.ROSInterruptException:
		rospy.loginfo("Interrupted, exiting ...")	

	rospy.loginfo("Success!")
	return client.get_result()

if __name__ == '__main__':
	try:
		rospy.init_node('go_to_point_client_py')
		result = go_to_goal_client()
	except rospy.ROSInterruptException:
		print("Interrupted")


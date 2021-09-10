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

	goal = GoToGoalGoal()

	goal.goal_pose.x = 1.0
	goal.goal_pose.y = 1.0
	goal.goal_pose.theta = 0.0

	client.send_goal(goal)

	client.wait_for_result()

	return client.get_result()

if __name__ == '__main__':
	try:
		print("start")
		rospy.init_node('go_to_point_client_py')
		result = go_to_goal_client()
	except rospy.ROSInterruptException:
		print("Interrupted")


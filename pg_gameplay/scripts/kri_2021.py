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


	rospy.loginfo("Input Goal:")
	goal = GoToGoalGoal()

	goal.goal_pose.x = float(input("Pose x: "))
	goal.goal_pose.y = float(input("Pose y: "))
	goal.goal_pose.theta = float(input("Pose theta: "))

	client.send_goal(goal)

	rospy.loginfo("Waiting for result ...")
	client.wait_for_result()

	if rospy.is_shutdown():
		rospy.loginfo("Interrupted, exiting ...")
		client.cancel_goal()
	else:
		rospy.loginfo("Success!")
		return client.get_result()

if __name__ == '__main__':
	try:
		rospy.init_node('go_to_point_client_py')
		result = go_to_goal_client()
	except rospy.ROSInterruptException:
		print("Interrupted")


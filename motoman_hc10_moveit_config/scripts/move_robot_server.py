#!/usr/bin/env python
from os import wait
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from motoman_hc10_moveit_config.srv import Robot_move
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

def handler_robot_move(msg):
    pose_goal = msg.Pose
    print("Move robot to : " + str(pose_goal))

    group.set_pose_target(pose_goal)

    plan = group.plan()

    if plan.joint_trajectory.joint_names == [] :
        print(false)
    else :
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

def move_robot_server():
    rospy.init_node('move_robot_server')
    s = rospy.Service('move_robot', Robot_move, handler_robot_move)

    print("Server move robot ready !")

if __name__ == "__main__":
    move_robot_server()
    rospy.spin()
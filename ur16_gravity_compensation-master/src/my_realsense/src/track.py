#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
import moveit_commander
from controller import Controller


def callback(msg):
    goal = PoseStamped()

    goal.pose.position.x = 0.3  # TODO
    goal.pose.position.y = msg.y
    goal.pose.position.z = msg.z

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.0

    return goal


def control():

    # TODO
    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    print(2)
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        print(3)
        robot = moveit_commander.RobotCommander()
        print(1)
        # scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("manipulator")

        # goal position of tool frame
        goal = rospy.Subscriber("/ar_tag_track/trans", Vector3, callback)
        print(goal)

        raw_input("Press <Enter> to generate plan")

        # generate plan
        group.set_pose_target(goal)
        plan = group.plan()

        #controller
        controller = Controller(Kp, Ki, Kd, Kw, robot, "/joint_group_vel_controller/command")
        if not controller.execute_path(plan):
            raise Exception("Execution failed")
    except Exception as e:
        print(e)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        control()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String

import moveit_commander
from geometry_msgs.msg import PoseStamped
import endeffectorOffset as eff_offsetCal

def robotStatePublisher():
    pub = rospy.Publisher('endEffectorPose', PoseStamped, queue_size=10)
    rospy.init_node('robotStatePublisher', anonymous=True)


    moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()
    group_name = "manipulator"    
    move_group = moveit_commander.MoveGroupCommander(group_name)

    

    rate = rospy.Rate(100) # 100hz
    previousTime = rospy.get_rostime()
    
    effPoseMsg = PoseStamped()
    while not rospy.is_shutdown():    

        temp_RobotPose = move_group.get_current_pose().pose
        effPoseMsg.pose = eff_offsetCal.endEffectorPoseFromRobot(temp_RobotPose) # I think the period of reading it is set by the "/joint_states" rate               
        # print thisTime-previousTime
        effPoseMsg.header.stamp= rospy.Time.now()
        

        pub.publish(effPoseMsg)
        # rate.sleep()

if __name__ == '__main__':
    try:
        robotStatePublisher()
    except rospy.ROSInterruptException:
        pass

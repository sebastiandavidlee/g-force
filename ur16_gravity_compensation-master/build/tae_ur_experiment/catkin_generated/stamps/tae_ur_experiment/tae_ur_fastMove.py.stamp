#!/usr/bin/env python

import os, sys
import rospy
from moveGroupInterface_Tae import MoveGroupInterface
from scipy.io import savemat
from datetime import datetime
import pandas as pd
import re
import subprocess
import numpy as np


from tae_psoc.msg import cmdToPsoc
from edg_data_logger.srv import *
from std_msgs.msg import String



    


def main():
  START_CMD = 2
  IDLE_CMD = 3

  LED_ON = 'ln'
  LED_OFF = 'lf'
  BLE_CLICK = 'b'
  BLINK_LED = 's'
  STOP_PSOC = 'i'
  PWM_SETTING = 'p' + chr(100) + chr(100) # p + Freq + Duty%
  PWM_ON = 'n'
  PWM_OFF = 'f'
  try:
    print "Position Test"
    rospy.init_node('tae_ur_fastmove')

    rospy.wait_for_service('data_logging')
    dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)

    sensorCMD_Pub = rospy.Publisher('cmdToPsoc', cmdToPsoc, queue_size=10)
    msg2Sensor = cmdToPsoc()

    miscCMD_Pub = rospy.Publisher('cmdToMics', String, queue_size= 10)
    msg2Misc = ""


    dataLoggerEnable(True) # Start Data Logging

    
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface UR_Interface"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Begin the tutorial by setting up the moveit_commander ..."    
    UR_Interface = MoveGroupInterface()
    # print "============ Add a sphere to the planning scene ..."        
    # UR_Interface.add_sphere()
    # print "============ Attach Sphere ..."    
    # UR_Interface.attach_sphere()        
    # print "============ Initial Joint State ..."    

    # UR_Interface.go_to_initial_joint_state()

    # print "=== Add Table and environment constraints"
    # big_table_size      = (INCHES_TO_M(60), INCHES_TO_M(30), INCHES_TO_M(36.25))    
    # big_table_position  = (-INCHES_TO_M(60/2 - 5),  INCHES_TO_M(0), -INCHES_TO_M(36.25/2 + 0.5))    
    # UR_Interface.add_box("big_table",big_table_position, big_table_size)

    # UR_Interface.add_WallBondary(Dx=INCHES_TO_M(-5), Dy=INCHES_TO_M(30), Dz=INCHES_TO_M(-0.5), Lx=INCHES_TO_M(30), Ly=INCHES_TO_M(-60), Lz=INCHES_TO_M(30))
    # UR_Interface.wait_for_state_update()

    print "== Go to Disengage ====="

    print UR_Interface.go_to_disengagePose_simple()

    print "============ Press `Enter` to execute a Start Pose"
    raw_input()
        # Start Vacuum and Reading
    miscCMD_Pub.publish(STOP_PSOC)
    rospy.sleep(0.1)
    #Turn on the Vacuum
    miscCMD_Pub.publish(PWM_SETTING)
    rospy.sleep(0.1)
    miscCMD_Pub.publish(PWM_ON)
    rospy.sleep(0.1)
    
    initialPosition = UR_Interface.engaging_endEffector_Position;
    # initialPosition[2] += 0.1;
    
    UR_Interface.go_to_Position(initialPosition, 0.3)


    print "============ Press `Enter` to start Vacuum and Sensor Reading"
    raw_input()
    UR_Interface.stop_and_clear()



    
    # Start Sensor Recording
    msg2Sensor.cmdInput = START_CMD
    sensorCMD_Pub.publish(msg2Sensor)

    print "============ Press `Enter` to go TargetPose"
    raw_input()
    targetPosition = initialPosition    
    targetPosition[2] += 0.05
    UR_Interface.go_to_Position(targetPosition, 0.3)

    print "============ Press `Enter` to Stop robot"
    raw_input()
    UR_Interface.stop_and_clear()

    print "============ Press `Enter` to end Vacuum and data recording"
    raw_input()    
    print(dataLoggerEnable(False)) # Stop Data Logging

    miscCMD_Pub.publish(PWM_OFF)
    rospy.sleep(0.1)    
    miscCMD_Pub.publish(STOP_PSOC)
    rospy.sleep(0.1)    
    
    msg2Sensor.cmdInput = IDLE_CMD
    sensorCMD_Pub.publish(msg2Sensor) # Stop Recording

    print "============ Press `Enter` to Back to disengage"
    raw_input()
    print UR_Interface.go_to_disengagePose_simple()


    print "============ Python UR_Interface demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':  
  main()



#!/usr/bin/env python2

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

def checkForceOutput():
    # First Check all CSV files in /tmp/ and bring them in as a variable  
  fileList = []
  for file in os.listdir("/tmp"):    
    if file.endswith(".csv") and file.find("netft"):
        fileList.append(os.path.join("/tmp", file))
        
  thisFT_CSV = sorted(fileList)[-1]
  df=pd.read_csv(thisFT_CSV)
  dataArray = df.values

  xDiffer = np.amax(dataArray[:,1]) - np.amin(dataArray[:,1])
  yDiffer = np.amax(dataArray[:,2]) - np.amin(dataArray[:,2])
  zDiffer = np.amax(dataArray[:,3]) - np.amin(dataArray[:,3])

  print "Force Diff in x, y, z"
  print xDiffer
  print yDiffer
  print zDiffer





    


def main():
  try:
    print "Position Test"
    rospy.init_node('tae_ur_run')

    rospy.wait_for_service('data_logging')
    dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)

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

    print "============ Press `Enter` to execute a Engage Pose"
    raw_input()
    
    
    if not UR_Interface.go_to_engagePose():
          UR_Interface.go_to_engagePose()


    print "============ Press `Enter` to execute DisengagePose"
    raw_input()
    print UR_Interface.go_to_disengagePose_simple()
    
  
    print(dataLoggerEnable(False)) # Stop Data Logging
    rospy.sleep(1)
    checkForceOutput();

    print "============ Python UR_Interface demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':  
  main()



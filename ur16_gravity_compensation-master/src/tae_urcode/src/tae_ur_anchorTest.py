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
import copy

import endeffectorOffset as eff_offsetCal
from tae_psoc.msg import cmdToPsoc
from netft_utils.srv import *
from edg_data_logger.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
import geometry_msgs.msg
import tf


prevIphoneFileName = ""

def INCHES_TO_MM(measure_in_inches):
    return 25.4*measure_in_inches

def MM_TO_M(measure_in_millimeters):
    return 0.001*measure_in_millimeters

def INCHES_TO_M(measure_in_inches):
    return 0.0254*measure_in_inches




def getLastMatFileSaved():
  ResultSavingDirectory = os.path.expanduser('~') + '/TaeExperiment/' + datetime.now().strftime("%y%m%d")
  if not os.path.exists(ResultSavingDirectory):
    os.makedirs(ResultSavingDirectory)
  fileList = []
  for file in os.listdir(ResultSavingDirectory):
    if file.endswith(".mat"):
      fileList.append(file)
  try:
    return sorted(fileList)[-1]
  except Exception as e:
      print e
  return "none"
  
  

def saveDataParams(args, engageShifts, translateVectors):
  global prevIphoneFileName
  rad2deg = 180.0 / np.pi

  ResultSavingDirectory = os.path.expanduser('~') + '/TaeExperiment/' + datetime.now().strftime("%y%m%d")
  if not os.path.exists(ResultSavingDirectory):
    os.makedirs(ResultSavingDirectory)

  #check if CSV Files are available
  tmp_dummyFolder = '/tmp/processed_csv'
  if not os.path.exists(tmp_dummyFolder):
    os.makedirs(tmp_dummyFolder)
   
  # First Check all CSV files in /tmp/ and bring them in as a variable  
  fileList = []
  for file in os.listdir("/tmp"):
      if file.endswith(".csv"):
          fileList.append(os.path.join("/tmp", file))

  savingDictionary = {}
  errorCount = 0
  for fileName in fileList:  
    try:    
      df=pd.read_csv(fileName)             
                          
      thisColumnName = df.columns.tolist()
      
      splitedList = re.split('_|\.', fileName)        
      thisTopicName = ''.join(splitedList[4:-1])        
      
      savingDictionary[thisTopicName+"_columnName"] = thisColumnName
      savingDictionary[thisTopicName+"_data"] = df.values
      #move to temparary folder    
      os.rename(fileName, tmp_dummyFolder + '/' + re.split('/',fileName)[-1])
    except Exception as e:
      print e
      errorCount +=1


  if errorCount > 1:
    raise Exception("!!!!!!!!!!!!!!!!!!---DataLoggerError---!!!!")



  depth = args.d * 1e-2
  modeNum = args.m
  engageShift = engageShifts[modeNum]
  translateVector = translateVectors[modeNum]
  
  
  savingDictionary["engageOffset"] = engageShift
  savingDictionary["transVector"] = translateVector
  savingDictionary["depth"] = depth
  savingDictionary["mode"] = modeNum
  savingDictionary["momentArm"] = args.ma * 1e-2
  savingDictionary["AddedWeight"] = args.w # in g
  savingDictionary["AnchorType"] = args.t
 


  savingFileName_noDir = 'DataLog_'+ '_'.join(splitedList[1:4])
  savingFileName = ResultSavingDirectory + '/' + savingFileName_noDir + '_anchor_' + args.t +'_depth_' + str(depth*1e2) + '_weight_' + str(args.w) +'_mArm_' + str(args.ma) +'_mode_' + str(modeNum)
  savemat(savingFileName, savingDictionary)

  return ResultSavingDirectory, savingFileName_noDir



# Clear all csv files in the tmp folder
def clearTmpFolder():
  fileList = []
  for file in os.listdir("/tmp"):
      if file.endswith(".csv"):
          fileList.append(os.path.join("/tmp", file))
  
  for fileName in fileList:
    os.remove(fileName)

averagingBuffer =[1]*10
inputIndex = 0
startAverage = False
averageFz = 0 

def callback(data):
  global thisForce  
  global averagingBuffer
  global inputIndex
  global startAverage
  global averageFz

  thisForce = data.wrench.force
  averagingBuffer[inputIndex] = thisForce
  inputIndex += 1
  if inputIndex == len(averagingBuffer):
    startAverage = True
    inputIndex= 0
  if startAverage:
    averageFz = 0
    for force in averagingBuffer:
      averageFz += force.z / len(averagingBuffer)

  # print thisForce









def main(args):
  global prevIphoneFileName
  global thisForce
  global averageFz

  depth = args.d * 1e-2
  modeNum = args.m

  engageShifts = [[0, 0, 0], [0, 3e-2, -5.5e-2], [0, 2e-2, -3.5e-2], [0, 3e-2, -5.5e-2]]
  translateVectors = [[0, 0, 5e-2], [0, 5e-2, 0], [0, 5e-2, 5e-2], [0, 5e-2, -3e-2]]

  engageShift = engageShifts[modeNum]
  translateVector = translateVectors[modeNum]
  

  FT_SimulatorOn = False

  print "Hi"

  deg2rad = np.pi / 180.0

  #######################################################################
  ###################### Initial Params #################################

  endEffectorOffsetLen = 0.2 # This will be the point of ortation
  engagePosition_Z0 = [-0.483, .0, 0.37] # When depth is 0 cm.

  initPosition = engagePosition_Z0[:]
  initPosition[2] += 0.35 

  engagePosition_Z0[0] += engageShift[0]
  engagePosition_Z0[1] += engageShift[1]
  engagePosition_Z0[2] += engageShift[2]


  rotateAxis = [0, 0, 1]  

  #######################################################################
  #######################################################################

  rospy.init_node('tae_ur_run')
  # rospy.init_node('tae_ur_run', anonymous=True)

  if FT_SimulatorOn:
    print "wait for FT simul"
    rospy.wait_for_service('start_sim')    
    netftSimCall = rospy.ServiceProxy('start_sim', StartSim)
    # netftSimCall(2,1, -0.1, 0)
    # print "Called"

  
  print "Wait for the data_logger"
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  

  dataLoggerEnable(False) # reset Data Logger just in case
  rospy.sleep(1)
  clearTmpFolder()

  try:
  
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface UR_Interface"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Begin the tutorial by setting up the moveit_commander ..."    
    UR_Interface = MoveGroupInterface()
    UR_Interface.engaging_endEffector_Position = engagePosition_Z0

    print "============ Add a sphere to the planning scene ..."        
    # UR_Interface.add_sphere()
    print "============ Attach Sphere ..."    
    # UR_Interface.attach_sphere()        
    print "============ Initial Joint State ..."    

    # UR_Interface.go_to_initial_joint_state()


    
    # if not len(UR_Interface.scene.get_known_object_names()) == 5:
    #   print "=== Add Table and environment constraints"
    #   big_table_size      = (INCHES_TO_M(60), INCHES_TO_M(30), INCHES_TO_M(36.25))    
    #   big_table_position  = (-INCHES_TO_M(60/2 - 5),  INCHES_TO_M(0), -INCHES_TO_M(36.25/2 + 0.5))    
    #   UR_Interface.add_box("big_table",big_table_position, big_table_size)

    #   FTIR_table_size      = (INCHES_TO_M(10), INCHES_TO_M(10), UR_Interface.engaging_endEffector_Position[2]-0.10)    
    #   FTIR_table_position  = (UR_Interface.engaging_endEffector_Position[0],  UR_Interface.engaging_endEffector_Position[1], FTIR_table_size[2]/2)    
    #   UR_Interface.add_box("FTIR_Setup",FTIR_table_position, FTIR_table_size)

    #   UR_Interface.add_WallBondary(Dx=INCHES_TO_M(-5), Dy=INCHES_TO_M(30), Dz=INCHES_TO_M(-0.5), Lx=INCHES_TO_M(30), Ly=INCHES_TO_M(-60), Lz=INCHES_TO_M(30))
    #   UR_Interface.wait_for_state_update()

    print "== Go to Disengage ====="
    raw_input()
    initRobotPose = copy.deepcopy(eff_offsetCal.robotPoseFromEndEffector(UR_Interface.get_engageEndEffectorPose()))
    initRobotPose.position.z += 0.25
    if not UR_Interface.go_to_PoseGradually(initRobotPose,speedScale=0.6, wantWait=True):
      UR_Interface.go_to_PoseGradually(initRobotPose,speedScale=0.6, wantWait=True)
    # print UR_Interface.go_to_disengagePose_simple()
    rospy.sleep(2)


    print "============ Press `Enter` to go to enage position..."
    raw_input()
    newEngagePosition = engagePosition_Z0[:]
    UR_Interface.engaging_endEffector_Position = newEngagePosition
    
    
    engageRobotPose = eff_offsetCal.robotPoseFromEndEffector(UR_Interface.get_engageEndEffectorPose())
    
      
    
    # if not UR_Interface.go_to_PoseGradually(engageRobotPose,speedScale=0.2, wantWait=True):
    #   UR_Interface.go_to_PoseGradually(engageRobotPose,speedScale=0.2, wantWait=True)

    waypoints, rotatedTargetRobotPose = UR_Interface.getRotatedRobotPose(copy.deepcopy(engageRobotPose),deg2rad*(60.0),[0,0,0],rotateAxis)    
    if not UR_Interface.go_to_PoseGradually(rotatedTargetRobotPose,speedScale=0.5, wantWait=True):
      print UR_Interface.go_to_PoseGradually(rotatedTargetRobotPose,speedScale=0.5,wantWait=True)


    # print "============ Press `rotate axis` to go to start motion..."
    # raw_input()
    # waypoints, engageRobotPose_rotated = UR_Interface.getRotatedRobotPose(copy.deepcopy(UR_Interface.move_group.get_current_pose().pose),deg2rad*(60.0),[0,0,0],rotateAxis)  
    # if not UR_Interface.go_to_PoseGradually(engageRobotPose_rotated,speedScale=0.5, wantWait=True):
    #   print UR_Interface.go_to_PoseGradually(engageRobotPose_rotated,speedScale=0.5,wantWait=True)

    print "============ Press `Enter` to go to start motion..."
    raw_input()
    # UR_Interface.move_group.get_current_pose().pose
    
    # translateVector = [0, 0, 5e-2]
    goalPose = copy.deepcopy(UR_Interface.move_group.get_current_pose().pose)
    goalPose.position.x += translateVector[0]
    goalPose.position.y += translateVector[1]
    goalPose.position.z += translateVector[2]

    #Enable data logging
    clearTmpFolder()
    dataLoggerEnable(True) # Start Data Logging
    rospy.sleep(1)
    
    moveSpeedScale = 0.01
    
      

    motionSuccess = UR_Interface.go_to_PoseGradually(goalPose,speedScale=moveSpeedScale, wantWait=False)
    print motionSuccess
    if not motionSuccess:
        print UR_Interface.go_to_PoseGradually(goalPose,speedScale=moveSpeedScale,wantWait=False)

    # rospy.sleep(1)
    print "============ Press `Enter` to stop recording..."
    raw_input()

    print(dataLoggerEnable(False)) # Stop Data Logging
    
    rospy.sleep(0.1)

    UR_Interface.move_group.stop()
    UR_Interface.move_group.clear_pose_targets()  
      
    # Check all the files and save it to data folder
    savingFolder, savingFileName = saveDataParams(args, engageShifts, translateVectors)



    print "============ Press `Enter` to go to disengage Position..."
    raw_input()
    if not UR_Interface.go_to_PoseGradually(initRobotPose,speedScale=0.8, wantWait=True):
      UR_Interface.go_to_PoseGradually(initRobotPose,speedScale=0.8, wantWait=True)



    
 
      

    print "============ Python UR_Interface demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--d', type=float, help='depth_cm', default=0.0)
  parser.add_argument('--m', type=int, help='testMode', default=0)
  parser.add_argument('--w', type=float, help='weightOn', default=0.0)
  parser.add_argument('--ma', type=float, help='momentArm in cm', default=11.0)
  parser.add_argument('--t', type = str, help= 'anchorType', default = 'disk')


  args = parser.parse_args()    
 
 
  main(args)
  # main(depth, engageShifts[mode], translateVectors[mode], mode)


    



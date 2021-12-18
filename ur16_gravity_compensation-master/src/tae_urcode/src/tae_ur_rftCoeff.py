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
  
  

def saveDataParams(args, betaDList, gammaDList):
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

      # try:
      #     df=pd.read_csv(fileName)             
                              
      #     thisColumnName = df.columns.tolist()
          
      #     splitedList = re.split('_|\.', fileName)        
      #     thisTopicName = ''.join(splitedList[4:-1])        
          
      #     savingDictionary[thisTopicName+"_columnName"] = thisColumnName
      #     savingDictionary[thisTopicName+"_data"] = df.values
      # except Exception as e:
      #     print(fileName + ": "+ str(e))
      #     pass
      # #move to temparary folder    
      # os.rename(fileName, tmp_dummyFolder + '/' + re.split('/',fileName)[-1])
  if errorCount > 1:
    raise Exception("!!!!!!!!!!!!!!!!!!---DataLoggerError---!!!!")
  
  intruderType = args.type
  gammaIndex = args.gI
  betaIndex = args.bI
  depth = args.d * 1e-2

  if intruderType == 0:
    savingDictionary["intruder"] = 'horizontal'
  elif intruderType == 1:
    savingDictionary["intruder"] = 'vertical'

  savingDictionary["beta"] = betaDList[betaIndex]
  savingDictionary["gamma"] = gammaDList[gammaIndex]
  savingDictionary["depth"] = depth


  savingFileName_noDir = 'DataLog_'+ '_'.join(splitedList[1:4])
  savingFileName = ResultSavingDirectory + '/' + savingFileName_noDir + '_intruder_' + str(intruderType) + '_beta_' + str(betaDList[betaIndex]) + '_gamma_' + str(gammaDList[gammaIndex])
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

  FT_SimulatorOn = False

  print "Hi"

  deg2rad = np.pi / 180.0

  #######################################################################
  ###################### Initial Params #################################
  intruderType = args.type
  gammaIndex = args.gI
  betaIndex = args.bI
  depth = args.d * 1e-2

  betaDList = [0, -30, 30, -60, 60, 90]
  gammaDList = [90, 75, 60, 45, 30, 15, 0] # Angle will be intruding and extruding in the same Path

  if betaIndex < 0:
    rotateAngle = 180 * deg2rad
    rotateAxis = [0, 0, 1]
  else:
    rotateAngle = 0 * deg2rad
    rotateAxis = [0, 0, 1]


  # if intruderType == 0: #Horizontal Intruder
  #   if betaIndex > 0:
  #     print "Wrong Beta Index"
  #     return    
  #   rotateAngle = betaDList[betaIndex] * deg2rad
  # elif intruderType == 1:
  #   if betaIndex < 3:
  #     print "Wrong Beta Index"
  #     return 
  #   thisangle = betaDList[betaIndex] - 90
  #   if thisangle < -90:
  #     thisangle += 180

  #   rotateAngle = thisangle* deg2rad
  # else:
  #   print "Wrong Intruder Index"
  #   return  
  # print "Rotated Angle"
  # print rotateAngle
  # rotateAxis = [-1, 0, 0]


  if gammaIndex == 0:
    translateVector = [0,0,-depth]
  elif gammaIndex == 6:
    translateVector = [0,8e-2,0]
  elif gammaIndex > 6 or gammaIndex < 0:
    print("Wrong gamma Index")
    return
  else:
    gamma = gammaDList[gammaIndex] * deg2rad
    translateVector = [0,depth/np.tan(gamma),-depth]


  
  endEffectorOffsetLen = 0.144 # This will be the point of ortation
  # engagePosition_Z0 = [-0.65, .0, 0.36] # When depth is 0 cm.

  engagePosition_Z0 = [-0.83, -.15, 0.34] # When depth is 0 cm. unit is in m

  hoveringHeight = -3e-2


  

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

    print "== Go to Disengage ====="
    # raw_input()
    print UR_Interface.go_to_disengagePose_simple()
    rospy.sleep(2)

    print "============ Press `Enter` to go search Initial Pose a movement using a pose goal ..."
    raw_input()
    UR_Interface.engaging_endEffector_Position = engagePosition_Z0 + [0, 0, hoveringHeight]
    
    
    engageRobotPose = eff_offsetCal.robotPoseFromEndEffector(UR_Interface.get_engageEndEffectorPose())

    # if intruderType == 1 or intruderType == 3:
    #   waypoints, engageRobotPose = UR_Interface.getRotatedRobotPose(copy.deepcopy(engageRobotPose),45*deg2rad,[0, 0, 0],[0, 0, 1])  


    waypoints, rotatedInitRobotPose = UR_Interface.getRotatedRobotPose(copy.deepcopy(engageRobotPose),rotateAngle,[0, 0, 0],rotateAxis)
    # waypoints, rotatedInitRobotPose = UR_Interface.getRotatedRobotPose(copy.deepcopy(engageRobotPose),rotateAngle,translateVector,rotateAxis)
    if not UR_Interface.go_to_PoseGradually(rotatedInitRobotPose,speedScale=0.5, wantWait=True):
      print UR_Interface.go_to_PoseGradually(rotatedInitRobotPose,speedScale=0.1,wantWait=True)

    rospy.sleep(1)

    
     #Enable data logging
    clearTmpFolder()
    dataLoggerEnable(True) # Start Data Logging
    print "!!!!!! Start collecting data  !!!!"
    rospy.sleep(1)

    
    # # go to z = 0
    # waypoints, zeroDepthPose = UR_Interface.getRotatedRobotPose(copy.deepcopy(rotatedInitRobotPose),0,[0, 0, -hoveringHeight],rotateAxis)
    # if not UR_Interface.go_to_PoseGradually(zeroDepthPose,speedScale=0.1, wantWait=True):
    #   print UR_Interface.go_to_PoseGradually(zeroDepthPose,speedScale=0.1,wantWait=True)    
    # rospy.sleep(2)

    print "============ Press `Enter` after adjusting to zero ..."
    raw_input()

    waypoints, deepPose = UR_Interface.getRotatedRobotPose(copy.deepcopy(UR_Interface.move_group.get_current_pose().pose),0,translateVector,rotateAxis)

    if not UR_Interface.go_to_PoseGradually(deepPose,speedScale=0.01, wantWait=True):
      print UR_Interface.go_to_PoseGradually(deepPose,speedScale=0.01,wantWait=True)

    print "============ Press `Enter` to start Extruding ..."
    raw_input()
    waypoints, extrudedPose = UR_Interface.getRotatedRobotPose(copy.deepcopy(UR_Interface.move_group.get_current_pose().pose),0,[-1*translateVector[0], -1*translateVector[1], -1*translateVector[2]],rotateAxis)

    if not UR_Interface.go_to_PoseGradually(extrudedPose,speedScale=0.01, wantWait=True):
      print UR_Interface.go_to_PoseGradually(extrudedPose,speedScale=0.01,wantWait=True)

    print dataLoggerEnable(False) # Stop Data Logging

     # Check all the files and save it to data folder
    savingFolder, savingFileName = saveDataParams(args, betaDList, gammaDList)

    print "== Go to Disengage ====="
    raw_input()
    if not UR_Interface.go_to_PoseGradually(rotatedInitRobotPose,speedScale=0.05, wantWait=True):
      print UR_Interface.go_to_PoseGradually(rotatedInitRobotPose,speedScale=0.05,wantWait=True)
    
    print "== Go to Disengage ====="
    raw_input()
    print UR_Interface.go_to_disengagePose_simple()

    rospy.sleep(2)

    
    
 
      

    print "============ Python UR_Interface demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--type', type=int, help='type of intruder, 0- horizontal, 30- 30deg fixture, 60 - 60deg, 90 - 90deg', default=0)
  parser.add_argument('--gI', type=int, help='Intruding gammaAngle Index', default=0)
  parser.add_argument('--bI', type=int, help='betaAngle_Index, 1 - postiive -1 --> negative', default=1)
  parser.add_argument('--d', type=float, help='endDepth_cm', default=8.0)

  args = parser.parse_args()    
  
  main(args)
  # main(depth, rotAngleList[mode], translateZList[mode])


    



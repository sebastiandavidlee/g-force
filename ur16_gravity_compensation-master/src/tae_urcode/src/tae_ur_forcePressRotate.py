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
  
  

def saveDataParams(rotateAngle,rotateAxis, dutyInfo, FzTarget):
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
  savingDictionary["rotateAngle_deg"] = rotateAngle * rad2deg
  savingDictionary["rotateAxis"] = rotateAxis
  savingDictionary["FreqDuty"] = dutyInfo
  savingDictionary["FzTarget"] = FzTarget


  savingFileName_noDir = 'DataLog_'+ '_'.join(splitedList[1:4])
  savingFileName = ResultSavingDirectory + '/' + savingFileName_noDir + '_Fz_' + str(FzTarget) + '_angle_' + str(rotateAngle * rad2deg) + '_' + str(dutyInfo[0]) + 'Hz' + str(dutyInfo[1]) +'duty'
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

def main(rotateAngle, FzTarget, freq, duty, iterNum):
  global prevIphoneFileName
  global thisForce
  global averageFz

  FT_SimulatorOn = False

  print "Hi"
  START_CMD = 2
  IDLE_CMD = 3

  LED_ON = 'ln'
  LED_OFF = 'lf'
  BLE_CLICK = 'b'
  BLINK_LED = 's'
  STOP_PSOC = 'i'  
  PWM_ON = 'n'
  PWM_OFF = 'f'
  deg2rad = np.pi / 180.0

  #######################################################################
  ###################### Initial Params #################################

  endEffectorOffsetLen = 0.2 # This will be the point of ortation
  engagePosition = [-0.483, .346, 0.3672]
  # engagePosition = [-0.3595, .335, 0.290]

  
  

  # engagePosition = [-0.3495, .287, 0.303]
  translateVector = [0,0,0]
  rotateAxis = [0,1,0]
  rotateAngle = rotateAngle * deg2rad

  # FzTarget = -0.5  
  pGain = -0.5e-3
  
  

  PWM_Freq_Duty  = [freq, duty]  
  PWM_SETTING = 'p' + chr(PWM_Freq_Duty[0]) + chr(PWM_Freq_Duty[1]) # p + Freq + Duty%
  # UR_Interface.go_to_Position(pressStartPosition, speedScale=0.3, wantWait=True)

  z_searchingDecrement = 0.001
  #######################################################################
  #######################################################################

  rospy.init_node('tae_ur_run')
  # rospy.init_node('tae_ur_run', anonymous=True)

  sensorCMD_Pub = rospy.Publisher('cmdToPsoc', cmdToPsoc, queue_size=10)
  msg2Sensor = cmdToPsoc()

  miscCMD_Pub = rospy.Publisher('cmdToMics', String, queue_size= 10)
  msg2Misc = ""

  # receive FT sensor read
  rospy.Subscriber("netft_data",WrenchStamped, callback)
  

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
    UR_Interface.engaging_endEffector_Position = engagePosition

    print "============ Add a sphere to the planning scene ..."        
    UR_Interface.add_sphere()
    print "============ Attach Sphere ..."    
    UR_Interface.attach_sphere()        
    print "============ Initial Joint State ..."    

    # UR_Interface.go_to_initial_joint_state()


    
    if not len(UR_Interface.scene.get_known_object_names()) == 5:
      print "=== Add Table and environment constraints"
      big_table_size      = (INCHES_TO_M(60), INCHES_TO_M(30), INCHES_TO_M(36.25))    
      big_table_position  = (-INCHES_TO_M(60/2 - 5),  INCHES_TO_M(0), -INCHES_TO_M(36.25/2 + 0.5))    
      UR_Interface.add_box("big_table",big_table_position, big_table_size)

      FTIR_table_size      = (INCHES_TO_M(10), INCHES_TO_M(10), UR_Interface.engaging_endEffector_Position[2]-0.10)    
      FTIR_table_position  = (UR_Interface.engaging_endEffector_Position[0],  UR_Interface.engaging_endEffector_Position[1], FTIR_table_size[2]/2)    
      UR_Interface.add_box("FTIR_Setup",FTIR_table_position, FTIR_table_size)

      UR_Interface.add_WallBondary(Dx=INCHES_TO_M(-5), Dy=INCHES_TO_M(30), Dz=INCHES_TO_M(-0.5), Lx=INCHES_TO_M(30), Ly=INCHES_TO_M(-60), Lz=INCHES_TO_M(30))
      UR_Interface.wait_for_state_update()

    print "== Go to Disengage ====="

    print UR_Interface.go_to_disengagePose_simple()
    rospy.sleep(2)

    print "============ Press `Enter` to go search Initial Pose a movement using a pose goal ..."
    # raw_input()

    
    
    engageRobotPose = eff_offsetCal.robotPoseFromEndEffector(UR_Interface.get_engageEndEffectorPose())
    waypoints, rotatedInitRobotPose = UR_Interface.getRotatedRobotPose(copy.deepcopy(engageRobotPose),rotateAngle,translateVector,rotateAxis)
    
    for trialIdx in range(iterNum):
      print "Go to no-rotate engagePose"
      # UR_Interface.go_to_engagePose()
      print UR_Interface.go_to_PoseGradually(engageRobotPose,speedScale=0.3,wantWait=True)
      print " Rotate it "
      if not UR_Interface.go_to_PoseGradually(rotatedInitRobotPose,wantWait=True):
        UR_Interface.go_to_PoseGradually(rotatedInitRobotPose,wantWait=True)

      rospy.sleep(1)

      
      
      # Get unit vector for of the pose
      poseQuaternion = [rotatedInitRobotPose.orientation.x, rotatedInitRobotPose.orientation.y, rotatedInitRobotPose.orientation.z, rotatedInitRobotPose.orientation.w]
      RMatrix = tf.transformations.quaternion_matrix(poseQuaternion)
      unitVector_search = RMatrix[0:3,0]

      
      lastMatFile = ""
      savingErrorCount = 0
      # i= -1

      clearTmpFolder()
      dataLoggerEnable(True) # Start Data Logging


      if not FT_SimulatorOn:
        thisMatFile = getLastMatFileSaved()      
        if lastMatFile is thisMatFile:      
          print "Mat file saving wrong!!!"        
          
        else:      
          lastMatFile = thisMatFile

      
      try:
        
        if FT_SimulatorOn:
          netftSimCall(2,1, -0.02, 0)
          rospy.sleep(1)

        zMove = 0.01 #initSearch 1cm
        goalRobotPose = copy.deepcopy(rotatedInitRobotPose)
        initFz = averageFz

        for zIdx in range(50):
          print "search Iterate: ", zIdx, "Zmove: ", zMove
          # Go to this Search Pose
          goalRobotPose.position.x  = rotatedInitRobotPose.position.x + zMove * unitVector_search[0]
          goalRobotPose.position.y  = rotatedInitRobotPose.position.y + zMove * unitVector_search[1]
          goalRobotPose.position.z  = rotatedInitRobotPose.position.z + zMove * unitVector_search[2]

          if UR_Interface.go_to_PoseGradually(goalRobotPose,speedScale=0.3, wantWait=True): #go to target pose success        
            rospy.sleep(1)
            thisFz = averageFz-initFz
            print thisFz

            if thisFz < 0.95 * FzTarget and thisFz > 1.05 * FzTarget:
              UR_Interface.go_to_PoseGradually(rotatedInitRobotPose,wantWait=True)
              break
            if thisFz > 0.5 * FzTarget: # if it is not touching anything
              zMove += 3e-3
            else:
              zMove += pGain * (FzTarget - thisFz)           

          

          UR_Interface.go_to_PoseGradually(rotatedInitRobotPose,wantWait=True)
          
          if rospy.is_shutdown():
            print "Oops"
            UR_Interface.go_to_disengagePose_simple()
            return

        if zIdx == 49:
          print "Fail to Find Fz"
          raise Exception("Error!!")

    

        print "Desired Pose"
        print goalRobotPose


        print "Suction Engage!"
        miscCMD_Pub.publish(STOP_PSOC)
        rospy.sleep(0.1)
        #Turn on the Vacuum
        miscCMD_Pub.publish(PWM_SETTING)
        rospy.sleep(0.1)
        miscCMD_Pub.publish(PWM_ON)
        rospy.sleep(0.1)

        
        # Start Sensor Recording
        msg2Sensor.cmdInput = START_CMD
        sensorCMD_Pub.publish(msg2Sensor)    
        #Turn LED ON
        miscCMD_Pub.publish(LED_ON)  
        rospy.sleep(0.5)      
        
        if not UR_Interface.go_to_PoseGradually(goalRobotPose,speedScale=0.3, wantWait=True):
          print UR_Interface.go_to_PoseGradually(goalRobotPose,speedScale=0.3, wantWait=True)
        
        rospy.sleep(5) # Time for collecting data
        
        #Turn off Vacuum        
        miscCMD_Pub.publish(PWM_OFF)
        rospy.sleep(0.3)
        # Start blinking FTIR LED to sync video, blinking will be recorded in the data file as well      
        miscCMD_Pub.publish(STOP_PSOC)
        rospy.sleep(0.5)
        
              
        UR_Interface.go_to_PoseGradually(rotatedInitRobotPose,wantWait=True)

        msg2Sensor.cmdInput = IDLE_CMD
        sensorCMD_Pub.publish(msg2Sensor) # Stop Recording
      
        
        print(dataLoggerEnable(False)) # Stop Data Logging
              

        # Check all the files and save it to data folder
        savingFolder, savingFileName = saveDataParams(rotateAngle,rotateAxis, PWM_Freq_Duty,FzTarget)    
      except Exception as e:
        print e

      except KeyboardInterrupt:
        print "Stopped"
        
      except rospy.ROSInterruptException:
        print "OopsOops"
        
          

      print "== Go to Disengage ====="
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
  parser.add_argument('--a', type=float, help='rotating angle', default=0.0)
  parser.add_argument('--f', type=float, help='force target', default=-0.5)
  parser.add_argument('--freq', type=int, help='force target', default=100)
  parser.add_argument('--duty', type=int, help='force target', default=100)
  parser.add_argument('--iter', type=int, help='force target', default=3)
  parser.add_argument('--auto', type=bool, help='force target', default=False)

  args = parser.parse_args()    
  angle = args.a
  force = args.f
  freq = args.freq
  duty = args.duty
  iterNum = args.iter
  autoMode = args.auto

  if not autoMode:
    print angle, force, freq, duty  
    main(angle, force, freq, duty, iterNum)
  else:
    angleList = [-30, -15, 0, 15, 30]
    freq_dutyList = [[100, 100],[100, 30], [30,30]]
    
    print "AutoMode!!"
    for [freq, duty] in freq_dutyList:
      for angle in angleList:      
        main(angle, force, freq, duty, iterNum)

    



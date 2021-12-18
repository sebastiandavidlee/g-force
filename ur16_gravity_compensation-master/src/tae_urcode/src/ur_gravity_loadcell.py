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
from math import pi, cos, sin

import endeffectorOffset as eff_offsetCal

from edg_data_logger.srv import *
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped
import geometry_msgs.msg
import tf


prevIphoneFileName = ""

loadcell_data = None

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
  

def saveDataParams(args):
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
  
  # initZ0 = args.Z0
  # relZend = args.relZ

  # savingDictionary["initZ0"] = initZ0 
  # savingDictionary["relZend"] = relZend

  savingFileName_noDir = 'DataLog_'+ '_'.join(splitedList[1:4])
  savingFileName = ResultSavingDirectory + '/' + savingFileName_noDir
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


def callbackLoadcell(message):
  global loadcell_data
  # print("Loadcell data: \"%s\"" %message.data)
  loadcell_data = message.data

def rosserialLoadcell():
  rospy.Subscriber("loadcell", Float32, callbackLoadcell)


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

  global loadcell_data

  FT_SimulatorOn = False
  deg2rad = np.pi / 180.0

  #######################################################################
  ###################### Initial Params #################################

  engagePosition = [0.340, .135, 0.90]

  #######################################################################
  #######################################################################

  rospy.init_node('tae_ur_run')
  # rospy.init_node('tae_ur_run', anonymous=True)

 
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

    print tf.transformations.quaternion_from_euler(pi/2,0,-pi,'sxyz')

    print "============ Initial Joint State ..."    
    # raw_input()
    # UR_Interface.go_to_initial_joint_state()


     #Enable data logging
    clearTmpFolder()
    dataLoggerEnable(True) # Start Data Logging
    print "!!!!!! Start collecting data  !!!!"
    rospy.sleep(1)

    # print "============ Press `Enter` to go Initial Pose"
    # raw_input() 
    # UR_Interface.go_to_disengagePose_simple()
    # goalPose = copy.deepcopy(UR_Interface.move_group.get_current_pose().pose)
    # print goalPose


    print "============ Press `Enter` to go Initial Pose"
    raw_input()
    UR_Interface.go_to_Position_simple(engagePosition, speedScale=0.03, wantWait=True)


    print "============ Press 'Enter' to get loadcell data"
    raw_input()
    # rover_weight = 1600
    # compensated_weight = rover_weight*0.5


    rosserialLoadcell()
    i = 0
    go = 1
    load_error = 0
    z_position = engagePosition

    while go:
      print(loadcell_data)
      print(compensated_weight)
      if loadcell_data == None:
        loadcell_data = 0
      else:
        load_error = compensated_weight - loadcell_data

      if load_error <= 10 and load_error >= - 10:
        print("complete")
        rospy.sleep(0.5)
        # break;
      elif load_error <= 0 and load_error > - 30:
        z_position[1] += load_error*-0.000001
        UR_Interface.go_to_Position_simple(z_position, speedScale=0.01, wantWait=False)
        print("move down slow")
        rospy.sleep(0.4)
      elif load_error <= 0:
        z_position[1] += load_error*-0.000001
        UR_Interface.go_to_Position_simple(z_position, speedScale=0.02, wantWait=False)
        print("move down")
        rospy.sleep(0.1)
      elif load_error >= 0 and load_error < 30:
        z_position[1] -= load_error*0.000001
        UR_Interface.go_to_Position_simple(z_position, speedScale=0.01, wantWait=False)
        print("move up slow")
        rospy.sleep(0.4)
      elif load_error >= 0:
        z_position[1] -= load_error*0.000001
        UR_Interface.go_to_Position_simple(z_position, speedScale=0.02, wantWait=False)
        print("move up")
        rospy.sleep(0.1)
      print("process")
      i += 1
      if i > 400:
        go = 0
        i = 0
        print("complete loop")
    return


    print "============ Press `Enter` to follow simulated AR Tag"
    i = 0
    go = 1
    ArTagPosition = engagePosition

    raw_input()
    while go:
      UR_Interface.go_to_Position_simple(ArTagPosition, speedScale=1, wantWait=True)
      ArTagPosition[0] -= 0.01
      i += 1

      if i > 5:
        go = 0
        i = 0


    print "============ Press `Enter` to move other location"
    raw_input()
    engagePosition = ArTagPosition
    engagePosition[0] += 0.05
    UR_Interface.go_to_Position_simple(engagePosition, speedScale=1, wantWait=True)



    rospy.sleep(2)
    print dataLoggerEnable(False) # Stop Data Logging

     # Check all the files and save it to data folder
    savingFolder, savingFileName = saveDataParams(args)

    print "== Check Saved file hit enter ====="
    raw_input()  

    import checkSavedMatFile

    rospy.sleep(2)

    print "============ Python UR_Interface demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  # import argparse
  # parser = argparse.ArgumentParser()
  # parser.add_argument('--Z0', type=float, help='start Z0 before press', default=0.55)
  global environment
  global compensation_const
  global compensated_weight

  rover_weight = float(sys.argv[1])
  environment = sys.argv[2]
  print(rover_weight)
  print(environment)

  if environment == 'Mars':
    compensation_const = 0.379
  elif environment == 'Moon':
    compensation_const = 0.165
  elif environment == 'Vinus':
    compensation_const = 0.905
  elif environment == 'Europa':
    compensation_const = 0.134
  elif environment == 'Pluto':
    compensation_const = 0.063

  compensated_weight = rover_weight *(1- compensation_const)
  print(compensated_weight)

  import argparse
  parser = argparse.ArgumentParser()
  # args = parser.parse_args()      
  args, unknown = parser.parse_known_args()

  main(args)
  # main(depth, rotAngleList[mode], translateZList[mode])
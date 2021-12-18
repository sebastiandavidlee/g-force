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
  
  initZ0 = args.Z0
  relZend = args.relZ


  savingDictionary["initZ0"] = initZ0 
  savingDictionary["relZend"] = relZend
  


  savingFileName_noDir = 'DataLog_'+ '_'.join(splitedList[1:4])
  savingFileName = ResultSavingDirectory + '/' + savingFileName_noDir + '_relZend_' + str(relZend) 
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



def main(args, engagePosition_Z0):
  global prevIphoneFileName
  global thisForce
  global averageFz

  FT_SimulatorOn = False

  print "Hi"

  deg2rad = np.pi / 180.0

  #######################################################################
  ###################### Initial Params #################################
  initZ0 = args.Z0
  relZend = args.relZ
  speedSet = args.s

  
  endEffectorOffsetLen = 0.144 # This will be the point of rotation
  # engagePosition_Z0 = [-0.65, .0, 0.36] # When depth is 0 cm.

  

 
  

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


    
     #Enable data logging
    clearTmpFolder()
    dataLoggerEnable(True) # Start Data Logging
    print "!!!!!! Start collecting data  !!!!"
    rospy.sleep(1)

    print "============ Press `Enter` to go Initial Pose"
    raw_input() 
    UR_Interface.go_to_Position(engagePosition_Z0, speedScale=0.2, wantWait=True)

    # print "============ Press `Enter` to go End Pose"
    print "============ Press `Enter` to LOOP"
    raw_input() 

    x_travel_eachIter = 2.5e-2
    for i in range(0,5):        
      if i > 0:
        engagePosition_Z0[0] += x_travel_eachIter
        UR_Interface.go_to_Position(engagePosition_Z0, speedScale=speedSet, wantWait=True)
      engagePosition_End = engagePosition_Z0[:]
      engagePosition_End[-1] = engagePosition_Z0[-1] + relZend
      UR_Interface.go_to_Position(engagePosition_End, speedScale=speedSet, wantWait=True)
      UR_Interface.go_to_Position(engagePosition_Z0, speedScale=speedSet, wantWait=True)
        



    print dataLoggerEnable(False) # Stop Data Logging

     # Check all the files and save it to data folder
    savingFolder, savingFileName = saveDataParams(args)
    
    
    
    print "== Go to Disengage ====="
    raw_input()
    print UR_Interface.go_to_disengagePose_simple()

    import checkSavedMatFile

    rospy.sleep(2)

    
    
 
      

    print "============ Python UR_Interface demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--Z0', type=float, help='start Z0 before press', default=0.55)
  parser.add_argument('--relZ', type=float, help='relative end Z from the start', default=-0.03)
  parser.add_argument('--s', type=float, help='speed [0, 1]', default=0.05)
  args = parser.parse_args()    
  engagePosition_Z0 = [-0.83, -.15, args.Z0] # When depth is 0 cm. unit is in m
  
  main(args,engagePosition_Z0)
  # main(depth, rotAngleList[mode], translateZList[mode])


    



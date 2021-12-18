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
from netft_utils.srv import *
from edg_data_logger.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped


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
  
  

def saveDataParams(index, motionList, dutyInfo, FzThres):
  global prevIphoneFileName

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

  savingDictionary["this_motionIndex"] = index
  savingDictionary["motionList"] = motionList
  savingDictionary["FreqDuty"] = dutyInfo
  savingDictionary["FzThres"] = FzThres


  savingFileName_noDir = 'DataLog_'+ '_'.join(splitedList[1:4])
  savingFileName = ResultSavingDirectory + '/' + savingFileName_noDir + '_idx_' + str(index) + '_' + str(dutyInfo[0]) + 'Hz' + str(dutyInfo[1]) +'duty'
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

averagingBuffer =[1]*4
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

def main():
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

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    
    pressStartPosition = UR_Interface.engaging_endEffector_Position[:];
    pressStartPosition[0] += 0.012;
    pressStartPosition[1] += 0.0;
    pressStartPosition[2] += 0.02;
    
    desired_X = (np.arange(0, -0.035, -0.015) + pressStartPosition[0]).tolist()  
    # desired_Y = (np.arange(0, -0.020, -0.005) + pressStartPosition[1]).tolist()  
    # desired_Y = [pressStartPosition[1]]


    z_searchingDecrement = 0.001
    FzThres = -0.5
    PWM_Freq_Duty  = [30, 30]
    
    PWM_SETTING = 'p' + chr(PWM_Freq_Duty[0]) + chr(PWM_Freq_Duty[1]) # p + Freq + Duty%
    # UR_Interface.go_to_Position(pressStartPosition, speedScale=0.3, wantWait=True)

    


    lastMatFile = ""
    savingErrorCount = 0
    # i= -1
    
    iterateNum = len(desired_X)
    for i in range(0,iterateNum):
      clearTmpFolder()
      dataLoggerEnable(True) # Start Data Logging


      if not FT_SimulatorOn:
        thisMatFile = getLastMatFileSaved()      
        if lastMatFile is thisMatFile:      
          print "Mat file saving wrong!!!"        
          break;
        else:      
          lastMatFile = thisMatFile

      
      try:
        pressStartPosition[0] = desired_X[i] 
        # pressStartPosition[1] = desired_Y[i] 

        
        if not UR_Interface.go_to_Position(pressStartPosition, speedScale=0.3, wantWait=True):
          UR_Interface.go_to_Position(pressStartPosition, speedScale=0.3, wantWait=True)

        # Search for the Force Threshold
        desiredPose = pressStartPosition[:]      
        
        if FT_SimulatorOn:
          netftSimCall(2,1, -0.2, 0)
          rospy.sleep(1)
          # print "Called"
        
        # rospy.sleep(0.5)
        # initFz = averageFz
        
        # # desiredPose[2] = 0.3462
        
        # z_searchingDecrement = 0.001
        # for j in range(0,50):        
        #   desiredPose[2] -= z_searchingDecrement                
        #   UR_Interface.go_to_Position(desiredPose, speedScale= 0.3, wantWait=True)
        #   rospy.sleep(1)
        #   thisForceVal = averageFz-initFz
        #   print thisForceVal
        #   if thisForceVal < FzThres*0.95 and thisForceVal > FzThres*1.1:
        #     break
        #   elif thisForceVal > -0.1:
        #     z_searchingDecrement = 0.002
        #   elif thisForceVal > 0.5 * FzThres:
        #     z_searchingDecrement = 0.001
        #   elif thisForceVal > 0.95 * FzThres:
        #     z_searchingDecrement = 0.0006
        #   elif thisForceVal < 1.5 * FzThres:
        #     z_searchingDecrement = -0.001
        #   elif thisForceVal < 1.1 * FzThres:
        #     z_searchingDecrement = -0.0004


        #   if not UR_Interface.go_to_Position(pressStartPosition, speedScale=0.3, wantWait=True):
        #     rospy.sleep(0.1)
        #     UR_Interface.go_to_Position(pressStartPosition, speedScale=0.3, wantWait=True)

        #   if rospy.is_shutdown():
        #     print "Oops"
        #     return

        # if j is 50:
        #   print "Fail to find Fz"
        #   break

        print "Desired Pose"
        print desiredPose

        UR_Interface.go_to_Position(pressStartPosition, speedScale=1, wantWait=True)
        


        

        print("***********iterate motion List Index = %d / %d" %(i, iterateNum))
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

        UR_Interface.go_to_Position(desiredPose, speedScale=0.3, wantWait=True)
        


        # Start Video Recording
        rospy.sleep(5) # Time for iphone to set focus      
        
        #Turn off Vacuum        
        miscCMD_Pub.publish(PWM_OFF)
        rospy.sleep(0.3)
        # Start blinking FTIR LED to sync video, blinking will be recorded in the data file as well      
        miscCMD_Pub.publish(STOP_PSOC)
        rospy.sleep(0.5)
        
              
        UR_Interface.go_to_Position(pressStartPosition, speedScale=1, wantWait=True)

        msg2Sensor.cmdInput = IDLE_CMD
        sensorCMD_Pub.publish(msg2Sensor) # Stop Recording
      
        
        print(dataLoggerEnable(False)) # Stop Data Logging
              

        # Check all the files and save it to data folder
        savingFolder, savingFileName = saveDataParams(i, desired_X, PWM_Freq_Duty,FzThres)    
      except Exception as e:
        print e

      except KeyboardInterrupt:
        print "Stopped"
        break
      except rospy.ROSInterruptException:
        print "OopsOops"
        break
        

    print "== Go to Disengage ====="
    print UR_Interface.go_to_disengagePose_simple()

    
    
    print "============ Press `Enter` to detach the box from the Panda robot ..."
    raw_input()
    UR_Interface.detach_box()
    print "============ Press `Enter` to remove the box from the planning scene ..."    
    UR_Interface.remove_box()

      

    print "============ Python UR_Interface demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  main()



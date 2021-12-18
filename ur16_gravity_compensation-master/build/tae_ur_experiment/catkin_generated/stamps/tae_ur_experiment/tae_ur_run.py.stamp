#!/usr/bin/env python

import os, sys
import rospy
from moveGroupInterface_Tae import MoveGroupInterface
from scipy.io import savemat
from datetime import datetime
import pandas as pd
import re
import subprocess


from tae_psoc.msg import cmdToPsoc
from edg_data_logger.srv import *
from std_msgs.msg import String

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
  
  

def saveDataParams(index, motionList,seedNum):
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
    df=pd.read_csv(fileName)             
                        
    thisColumnName = df.columns.tolist()
    
    splitedList = re.split('_|\.', fileName)        
    thisTopicName = ''.join(splitedList[4:-1])        
    
    savingDictionary[thisTopicName+"_columnName"] = thisColumnName
    savingDictionary[thisTopicName+"_data"] = df.values
    #move to temparary folder    
    os.rename(fileName, tmp_dummyFolder + '/' + re.split('/',fileName)[-1])

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
    
  iphoneFileName = getIphoneLastFileName()
  if iphoneFileName == prevIphoneFileName:
    raise AssertionError("Iphone Record Fail")
  prevIphoneFileName = iphoneFileName
  print iphoneFileName

  savingDictionary["this_motionIndex"] = index
  savingDictionary["motionList"] = motionList
  savingDictionary["iphoneFileName"] = iphoneFileName;
  savingDictionary["seedNum"] = seedNum;

  savingFileName_noDir = 'DataLog_'+ '_'.join(splitedList[1:4])
  savingFileName = ResultSavingDirectory + '/' + savingFileName_noDir +'_seed_' + str(seedNum) + '_idx_' + str(index)
  savemat(savingFileName, savingDictionary)

  return ResultSavingDirectory, savingFileName_noDir


def getIphoneLastFileName():
  #Check iphone File Name  
  iphoneFolder = os.path.expanduser('~') + "/iphoneTest/DCIM"  
  iphoneFileList = []
  for subfolder in next(os.walk(iphoneFolder))[1]:  
    for file in os.listdir(iphoneFolder+"/"+subfolder):
        if file.endswith(".MOV"):
            iphoneFileList.append(file)
  return sorted(iphoneFileList)[-1]



# Clear all csv files in the tmp folder
def clearTmpFolder():
  fileList = []
  for file in os.listdir("/tmp"):
      if file.endswith(".csv"):
          fileList.append(os.path.join("/tmp", file))
  
  for fileName in fileList:
    os.remove(fileName)

def moveIPhoneFile(destinationFolder, savingFileName_noDir):
  #sudo apt install libimobiledevice6 libimobiledevice-utils
  #idevicepair pair
  #usbmuxd -f -v
  #ifuse /LOCAL_DIRECTORY
  #ifuse -u /LOCAL_DIRECTORY  # After it is done
  #fusermount -u /LOCAL_DIRECTORY #unmount
  # see https://gist.github.com/samrocketman/70dff6ebb18004fc37dc5e33c259a0fc

  
  iphoneFolder = os.path.expanduser('~') + "/iphoneTest/DCIM/100APPLE"  
  fileList = []
  for file in os.listdir(iphoneFolder):
      if file.endswith(".MOV"):
          fileList.append(file)
  thisFileName = sorted(fileList)[-1]
  print "iphoneFile: " + thisFileName
  
  #move this file to destination
  # os.rename(iphoneFolder + '/' + thisFileName, destinationFolder+'/'+savingFileName_noDir+'_'+thisFileName)
  
  
  subprocess.check_call(['mv', iphoneFolder + '/' + thisFileName, destinationFolder+'/'+savingFileName_noDir+'_'+thisFileName ])




def main(seedNum, iterStart):
  global prevIphoneFileName
  try:
    print "Hi"
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

    rospy.init_node('tae_ur_run')
    # rospy.init_node('tae_ur_run', anonymous=True)

    sensorCMD_Pub = rospy.Publisher('cmdToPsoc', cmdToPsoc, queue_size=10)
    msg2Sensor = cmdToPsoc()

    miscCMD_Pub = rospy.Publisher('cmdToMics', String, queue_size= 10)
    msg2Misc = ""

    


    print "Wait for the data_logger"
    rospy.wait_for_service('data_logging')
    dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)

    clearTmpFolder()

    
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface UR_Interface"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Begin the tutorial by setting up the moveit_commander ..."    
    
    
    UR_Interface = MoveGroupInterface(seedNum=seedNum) # seedNum 0 do not add random variation

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

      FTIR_table_size      = (INCHES_TO_M(10), INCHES_TO_M(10), UR_Interface.engaging_endEffector_Position[2]-0.03)    
      FTIR_table_position  = (UR_Interface.engaging_endEffector_Position[0],  UR_Interface.engaging_endEffector_Position[1], FTIR_table_size[2]/2)    
      UR_Interface.add_box("FTIR_Setup",FTIR_table_position, FTIR_table_size)

      UR_Interface.add_WallBondary(Dx=INCHES_TO_M(-5), Dy=INCHES_TO_M(30), Dz=INCHES_TO_M(-0.5), Lx=INCHES_TO_M(30), Ly=INCHES_TO_M(-60), Lz=INCHES_TO_M(30))
      UR_Interface.wait_for_state_update()

    print "== Go to Disengage ====="

    print UR_Interface.go_to_disengagePose_simple()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    
    iterateNum = len(UR_Interface.motionList)
    
    prevIphoneFileName = getIphoneLastFileName()
    print prevIphoneFileName
    
    lastMatFile = ""
    savingErrorCount = 0
    # i= -1
    i = iterStart
    i = i-1
    while i < iterateNum-1:
      i += 1
      thisMatFile = getLastMatFileSaved()      
      if lastMatFile == thisMatFile and savingErrorCount < 1:
        print "Mat file saving wrong, retry"        
        i -=1
        savingErrorCount += 1
      elif lastMatFile == thisMatFile and savingErrorCount >= 1:
        print "Mat file saving wrong!!!"        
        break;
      else:
        savingErrorCount = 0
        lastMatFile = thisMatFile


      try:
        dataLoggerEnable(True) # Start Data Logging

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

        if not UR_Interface.go_to_engagePose():
          UR_Interface.go_to_engagePose()


        # Start Video Recording
        rospy.sleep(2) # Time for iphone to set focus
        miscCMD_Pub.publish(BLE_CLICK)
        rospy.sleep(1)
        
        planSucceed = UR_Interface.plan_execute_cartesian_path(i)
        print("plan succeed %d" %planSucceed)
        
        rospy.sleep(1)
        #Turn off Vacuum        
        miscCMD_Pub.publish(PWM_OFF)
        rospy.sleep(0.1)
        # Start blinking FTIR LED to sync video, blinking will be recorded in the data file as well
        miscCMD_Pub.publish(BLINK_LED)
        rospy.sleep(1.5) # Time to record the blinking LED Pulse
        miscCMD_Pub.publish(STOP_PSOC)
        rospy.sleep(0.5)
        #Stop Video Recording
        miscCMD_Pub.publish(BLE_CLICK)
        rospy.sleep(0.5)
        
        if not UR_Interface.go_to_disengagePose():
          print "smooth Fail--> simple"
          UR_Interface.go_to_disengagePose_simple()

        msg2Sensor.cmdInput = IDLE_CMD
        sensorCMD_Pub.publish(msg2Sensor) # Stop Recording
        # miscCMD_Pub.publish('close') #close Comport        
        
        print(dataLoggerEnable(False)) # Stop Data Logging
        
        #Give some Time interval
        rospy.sleep(2)

        # Check all the files and save it to data folder
        savingFolder, savingFileName = saveDataParams(i, UR_Interface.motionList, seedNum)

        # Move the .MOV File from Iphone to local drive not to overflow the iphone storage
        # moveIPhoneFile(savingFolder, savingFileName)
        # rospy.sleep(3)

        if rospy.is_shutdown():
          print "Oops"
          return
      except Exception as e:
        print e

      except KeyboardInterrupt:
        print "Stopped"
        break
       
      

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # UR_Interface.display_trajectory(cartesian_plan)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    # UR_Interface.execute_plan(cartesian_plan)
    
    
    
    
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
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--s', type=int, help='randomSeed', default=0)
  parser.add_argument('--iter', type=int, help='IterStart', default=0)
  

  args = parser.parse_args()    
  seedNum = args.s
  iterStart = args.iter
  
  main(seedNum, iterStart)



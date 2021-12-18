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
  except:
    return "none"
  
  

def saveDataParams(r_move, runMode):
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
  
  iphoneFileName = getIphoneLastFileName()
  if iphoneFileName == prevIphoneFileName:
    raise Exception("Iphone Record Fail")
  prevIphoneFileName = iphoneFileName
  print iphoneFileName
  
  savingDictionary["iphoneFileName"] = iphoneFileName
  savingDictionary["speedParam_r"] = r_move
  savingDictionary["runMode"] = runMode
  
  savingFileName_noDir = 'DataLog_'+ '_'.join(splitedList[1:4])
  savingFileName = ResultSavingDirectory + '/' + savingFileName_noDir + '_' + runMode
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


def goToInit(r=0.2):  

  vname_reset = os.path.expanduser('~') + '/UR_FastMoving/ur5_go/ur5_vertical_fabric_reset.dat'    
  tstep = str(0.032/r) 
  executeFileName = os.path.expanduser('~') + '/UR_FastMoving/ur5_go/build/ur5_go'
  os.system(executeFileName + ' -h 10.0.0.1 -r 1.0 -t '+tstep+' '+vname_reset) # Should think if I need to use r = 1 or not.

def runFastMove(r=0.2):  
  vname = os.path.expanduser('~') + '/UR_FastMoving/ur5_go/ur5_vertical_fabric.dat'    
  tstep = str(0.032/r) 
  executeFileName = os.path.expanduser('~') + '/UR_FastMoving/ur5_go/build/ur5_go'
  os.system(executeFileName + ' -h 10.0.0.1 -r 1.0 -t '+tstep+' '+vname) # Should think if I need to use r = 1 or not.



def main(r_move, runMode):
  global prevIphoneFileName
  try:
    print "Hi"
    START_CMD = 2
    IDLE_CMD = 3

    LED_ON = 'ln'
    LED_OFF = 'lf'
    BLE_CLICK = 'b'
    BLINK_LED = 's'
    STOP_BLINKINGLED = 'c'
    STOP_PSOC = 'i'
    PWM_SETTING = 'p' + chr(100) + chr(100) # p + Freq + Duty%
    PWM_ON = 'n'
    PWM_OFF = 'f'

    rospy.init_node('tae_ur_fastrun')
    # rospy.init_node('tae_ur_run', anonymous=True)

    sensorCMD_Pub = rospy.Publisher('cmdToPsoc', cmdToPsoc, queue_size=10)
    msg2Sensor = cmdToPsoc()

    miscCMD_Pub = rospy.Publisher('cmdToMics', String, queue_size= 10)
    msg2Misc = ""

    


    print "Wait for the data_logger"
    rospy.wait_for_service('data_logging')
    dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)

    clearTmpFolder()


    print "== Go to Disengage ====="
    start_config = [-3.5766919294940394, -2.0647252241717737, 2.289108991622925, -1.7947500387774866, -1.5717080275165003, -0.43426639238466436]   
    end_config = [-3.3885870615588587, -1.4883397261248987, 1.838463306427002, -1.924002472554342, -1.5719955603228968, -0.24620420137514287]
    print runMode
    if runMode == 'fast':
      goToInit(r=0.7)
    elif runMode == 'slow':
      print "Initialize Interface"
      UR_Interface = MoveGroupInterface(seedNum=0)
      UR_Interface.go_to_goal_jointState(start_config,speed=0.3)




    prevIphoneFileName = getIphoneLastFileName()
    thisMatFile = getLastMatFileSaved()   
    
    print prevIphoneFileName

        
    dataLoggerEnable(True) # Start Data Logging
    
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
    
    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    rospy.sleep(1) # Time for iphone to set focus
    miscCMD_Pub.publish(BLE_CLICK)
    rospy.sleep(1)
    
    # Start blinking FTIR LED to sync video, blinking will be recorded in the data file as well
    miscCMD_Pub.publish(BLINK_LED)
    rospy.sleep(1.5) # Time to record the blinking LED Pulse
    miscCMD_Pub.publish(STOP_BLINKINGLED)
    rospy.sleep(0.2)
    #Turn LED ON
    miscCMD_Pub.publish(LED_ON)  
    rospy.sleep(2) 

    # ------------------------Run The Moving Code
    if runMode == 'fast':
      runFastMove(r=r_move)
    elif runMode == 'slow':
      UR_Interface.go_to_goal_jointState(end_config,speed=r_move)

    print("Done") # check if os.system waits until the program is done or not
    rospy.sleep(2) 
    miscCMD_Pub.publish(BLE_CLICK)
    rospy.sleep(1)
    print(dataLoggerEnable(False)) # Stop Data Logging

    msg2Sensor.cmdInput = IDLE_CMD
    sensorCMD_Pub.publish(msg2Sensor) # Stop Recording    
    miscCMD_Pub.publish(STOP_PSOC)

    # Check all the files and save it to data folder
    savingFolder, savingFileName = saveDataParams(r_move, runMode)
    print "============ Python UR_FastMove demo complete!"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()  
  parser.add_argument('--r', type=float, help='Scale of velocity and acceleration bounds', default=0.5)
  parser.add_argument('--m', type=str, help='sets mode fast or slow', default='fast')
  args = parser.parse_args()
  this_r = args.r  
  runMode = args.m

  print runMode

  if not (runMode == 'fast' or runMode == 'slow'):
    runMode = 'fast'

  print runMode

  print this_r
  main(float(this_r),runMode)



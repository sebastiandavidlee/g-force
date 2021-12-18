import os
from scipy.io import loadmat
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime


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
      print(e)
  return "none"


folderName = os.path.expanduser('~') + '/TaeExperiment/' + datetime.now().strftime("%y%m%d")
# fileName = "DataLog_2021_0204_164929_idx_0_30Hz30duty.mat"

mat_contents = loadmat(folderName + '/' + getLastMatFileSaved())

print( sorted(mat_contents.keys()) )

#%% Plot the data
plt.figure()

dataBuffer = mat_contents['endEffectorPose_data']
plt.clf()
plt.plot(dataBuffer[:,-3:])
plt.ylabel('Robot Position')
plt.xlabel('sample')
plt.grid()
plt.show(block=False)
plt.title('Robot Position')
plt.pause(1)

plt.close()

# plt.show()
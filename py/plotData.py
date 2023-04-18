import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import sys


def plotData(filename):
  print(" ")

  with open(filename,'r') as fp:
    for lineCount, line in enumerate(fp):
      pass
  lineCount +=1
  print('Total Lines: {}'.format(lineCount))

  df = pd.read_csv(filename, sep=',', header=None)
  x = np.array(df)[:,0]
  y = np.array(df)[:,1]
  print(x)
  print(y)
  # plt.plot(x)
  plt.plot(y)
  plt.plot(geometricFilter(y,8))
  plt.show()

def geometricFilter(data, bitShift):
  for i in range (1,len(data)):
    data[i] = (data[i-1] * (2**bitShift-1) + data[i])/(2**bitShift)
  return data
  

plotData(sys.argv[1])
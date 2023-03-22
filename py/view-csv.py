import pandas as pd
import numpy as np
from matplotlib import pyplot as plt


def viewcsv(filename):
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
  plt.plot(x)
  plt.plot(y/256)
  plt.show()
  

viewcsv("data/out/filter_test.c.led_off_test.csv")
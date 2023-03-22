import sys

dataIn = sys.argv[1]
dataDest = sys.argv[2]
if (len(sys.argv) == 4):
  varName = sys.argv[3]
else:
  varName = dataIn

outFile = open(dataDest,'a');
print(" ")

with open(dataIn,'r') as fp:
  for lineCount, line in enumerate(fp):
    pass
lineCount +=1
print('Total Lines: {}'.format(lineCount))

with open(dataIn, 'r') as data:
  outFile.write("uint16_t {}[{}] = {{".format(varName,lineCount))
  outFile.write(data.readline().replace("\r","").replace("\n",""))
  for line in data.readlines():
    outFile.write(",{}".format(line.replace("\n","").replace("\r","")))
  outFile.write("};\n\n")
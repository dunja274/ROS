#!/usr/bin/env python3

import sys

inputFile = sys.argv[1]
print("input file name: " + inputFile)
outputFile = sys.argv[2]
print("Output file name: " + outputFile)

x = input("Floating point number: ")
x = float(x.strip())

fi = open(inputFile, "r")
fo = open(outputFile, "w")

result=''

for l in fi:
    num = float(l.strip())+x
    print('{0} + {1} = {2}'.format(l.strip(),x, num))
    result += str(num)+'\n'

fo.write(result[:-1])

fi.close()
fo.close()
    
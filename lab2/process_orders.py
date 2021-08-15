#!/usr/bin/env python3

import sys

inputFile = sys.argv[1]
print("Processing " + inputFile)

orderCount = 0

fi = open(inputFile, "r")
fo = open("orders_report.txt", "w")

result=''
orderDict = {}

while fi.readline():
    n = fi.readline().strip()
    if(n == ''):
        break
    orderCount += 1
    for i in range(int(n)):
        line = (fi.readline()).split(':')
        if line[0].strip() not in orderDict:
            orderDict[line[0].strip()] = 0
        orderDict[line[0].strip()] += float(line[1].strip())


result += 'Number of orders:' + str(orderCount) + '\n'
result += 'Item totals:\n'

for x, y in orderDict.items():
  result += '  ' + x + ': ' + str(y) + '\n'

print(result[:-1])

fo.write(result[:-1])

fi.close()
fo.close()
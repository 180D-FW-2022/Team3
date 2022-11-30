import orTak
from time import sleep

sampleItemOrder=[("chicken sandwich", 4), ("milkshake", 5)]

tables = []

testNum = 1

for i in range(1,testNum+1):
    tables.append(orTak.OrTak(i))

for i in range(testNum):
    tables[i].testOrder(sampleItemOrder)
    sleep(0.05)
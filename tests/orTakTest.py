from time import sleep
import random
import orTak

sampleItemOrder=[("chicken sandwich", 4), ("milkshake", 5), ("chicken sandwich", 10), ("vegan burger", 5), ("chicken tenders burger", 5)]

tables = []
tn = []

testNum = 10
testNums = [1,2,3,4,5]

# for i in range(1,testNum+1):
for i in range(testNum):
    tables.append(orTak.OrTak(random.randint(1, testNum+1)))

for i in tables:
    print(sampleItemOrder)
    i.testOrder(sampleItemOrder)
    # sleep(0.05)
for i in tables:
    if i.tableNumber not in tn:
        tn.append(i.tableNumber)

print(tn)
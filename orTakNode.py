import orTak
import sys

tableNumber = sys.argv[1]
print("OrTak Table Number:", tableNumber)
tableOne = orTak.OrTak(tableNumber)
tableOne.takeOrder()

#run with 2>/dev/null
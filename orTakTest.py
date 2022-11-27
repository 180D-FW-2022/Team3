import orTak

sampleItemOrder=[("chicken sandwich", 4), ("milkshake", 5)]

tables = []

for i in range(4):
    tables.append(orTak.OrTak(i))

for i in range(4):
    tables[i].testOrder(sampleItemOrder)
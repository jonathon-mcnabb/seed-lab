import sys

with open('datafile.txt', 'r') as f:
    b = eval(f.read())

    maxVal = -sys.maxsize - 1
    minVal = sys.maxsize

    index = -1

    length = len(b)

    for i in range(length):
        if b[i] > maxVal:
            maxVal = b[i]
        if b[i] < minVal:
            minVal = b[i]
        if b[i] == 38:
            index = i

    b.sort()

    frequency = 1
    count = 0
    values = []
    for i in range(length-1):
        if b[i+1] == b[i]:
            frequency += 1
        else:
            if frequency > count:
                count = frequency
                values = []
                values.append(b[i])
            elif frequency == count:
                values.append(b[i])
            frequency = 1

    if frequency > count:
        count = frequency
        values = []
        values.append(b[i])
    elif frequency == count:
        values.append(b[i])

    evens = []
    for i in range(length):
        if b[i] % 2 == 0:
            evens.append(b[i])

    print('Max: ' + str(maxVal))
    print('Min: ' + str(minVal))
    print('38 is stored at index: ' + str(index))
    print('Most common: ' + str(values) + ' was repeated ' + str(count) + ' times.')
    print('Sorted: ' + str(b))
    print('Evens: ' + str(evens))

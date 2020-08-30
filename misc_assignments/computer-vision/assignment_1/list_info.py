"""List Info Parser

This script allows the user to input a data file and find the min, max, 
index of 38, most common values, the sorted array, and even numbers in the array.

This tool is designed to work with a .txt file that has an array in the 
following format: [x1, x2, ... , xn]

"""

import sys

# open file
with open('datafile.txt', 'r') as f:
    b = eval(f.read())
    # set initial maxVal to minimum
    # set initial minVal to maximum
    maxVal = -sys.maxsize - 1
    minVal = sys.maxsize

    index = -1

    length = len(b)

    # loop through datafile linearly to find max, min, and index of 38
    for i in range(length):
        if b[i] > maxVal:
            maxVal = b[i]
        if b[i] < minVal:
            minVal = b[i]
        if b[i] == 38:
            index = i

    # sort the list 
    b.sort()

    # every val exists at least once. freq starts at 1
    frequency = 1
    count = 0
    values = []
    # compare future (b[i+1) values to the current b[i]
    for i in range(length-1):
	# if they are equal, then the freq goes up by 1
        if b[i+1] == b[i]:
            frequency += 1
        else:
	    # if this val was more often than previous count, record it
            if frequency > count:
                count = frequency
                values = []
                values.append(b[i])
	    # if this val was equal to previous count, 
	    # record both prev val and current val
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

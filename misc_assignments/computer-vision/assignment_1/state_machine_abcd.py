
"""abcd checker

This script allows the user to input a string and will output 
'abcd is contained in the string' everytime the sequence abcd is found

"""
# state definition for state0
def state0(character):
    if character != 'a':
        return state0
    elif character == 'a':
        return statea

# state definition for stata
def statea(character):
    if character == 'a':
        return statea
    elif character == 'b':
        return stateb
    elif character != 'a' or character != 'b':
        return state0

# state definition for stateb
def stateb(character):
    if character == 'a':
        return statea
    elif character == 'c':
        return statec
    elif character != 'a' or character != 'c':
        return state0

# state definition for statec
def statec(character):
    if character == 'a':
        return statea
    elif character == 'd':
        return stated
    elif character != 'a' or character != 'd':
        return state0


# state definition for stated
def stated(character):
    print("abcd is contained in the string")
    if character == 'a':
        return statea
    elif character != 'a':
        return state0

# create a dictionary to describe the states. helpful for debugging
state_dictionary = {
    state0 : "State 0",
    statea : "State A",
    stateb : "State B",
    statec : "State C",
    stated : "State D"
}


state = state0 # initial state as pointer to state0 function
print("Type any string")
print("State machine will detect if abcd is contained in the string")
input_string = input("Enter String: ") # request input
input_string = list(input_string)
print("")

length = len(input_string)

for i in range(length): # Run until every character has been iterated over
    new_state = state(input_string[i]) # move to next state
    state = new_state # update control to next state
new_state = state(input_string[0]) # check last character
print("\nDone with state machine")

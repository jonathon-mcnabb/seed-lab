def state0(character):
    if character != 'a':
        return state0
    elif character == 'a':
        return statea

def statea(character):
    if character == 'a':
        return statea
    elif character == 'b':
        return stateb
    elif character != 'a' or character != 'b':
        return state0


def stateb(character):
    if character == 'a':
        return statea
    elif character == 'c':
        return statec
    elif character != 'a' or character != 'c':
        return state0


def statec(character):
    if character == 'a':
        return statea
    elif character == 'd':
        return stated
    elif character != 'a' or character != 'd':
        return state0


def stated(character):
    print("abcd is contained in the string")
    if character == 'a':
        return statea
    elif character != 'a':
        return state0

# create a dictionary to describe the states
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
input_string = input("Enter String: ")
input_string = list(input_string)

length = len(input_string)

for i in range(length): # Run until state is None
    new_state = state(input_string[i]) # launch state machine
    state = new_state # update the next state
new_state = state(input_string[0]) # launch state machine
print("Done with state machine")

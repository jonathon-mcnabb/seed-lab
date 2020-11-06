from statemachine import StateMachine

from main import share_angle
from main import share_points

angleConfirmed = False
pointsConfirmed = False

def init():
    # run any setup code here

def pass_angle():
    share_angle()
    if(angleConfirmed):
        newState = "points"
    else:
        newState = "angle"
    return newState

def pass_points():
    share_points()
def finish():

def default():


if __name__== "__main__":
    m = StateMachine()
    m.add_state("init", init)
    m.add_state("angle", pass_angle)
    m.set_start("points", pass_points)
    m.run()


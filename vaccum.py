import random
from HAL import HAL
from GUI import GUI

dir(GUI)

class State:
    NOP=-1
    START_SPIRAL=0
    SPIRAL=1
    START_FD=2
    FD=3
    START_BW=4
    BW=5
    START_LEFT=6
    LEFT=7
    
def check_sensors():
    laser=HAL.getLaserData()
    LEFT_ANGLE=180-30
    RIGHT_ANGLE=30
    DISTANCE=300
    left=min(laser[LEFT_ANGLE:])<DISTANCE
    right=min(laser[:RIGHT_ANGLE])<DISTANCE
    front=min(laser[RIGHT_ANGLE:LEFT_ANGLE])<DISTANCE
    return left,front,right
    

state=State.START_BW
state_variable=0

while True:
    print(check_sensors())
    state=State.NOP
    if(state==State.START_SPIRAL):
      state_variable=0#w
      state=State.SPIRAL
      
    if(state==State.SPIRAL):
      HAL.setV(0.1+state_variable/1000)
      HAL.setW(1)
      state_variable+=1
      if(state_variable>=5000):
        state=State.START_SPIRAL
    
    if(state==State.START_BW):
      state_variable=random.randrange(10, 50)
      state=State.BW
    if(state==State.BW):
      HAL.setV(-1)
      HAL.setW(0)
      state_variable-=1
      if(state_variable<=0):
        state=State.START_LEFT
    if(state==State.START_LEFT):
      state_variable=random.randrange(10, 50)
      state=State.LEFT
    if(state==State.LEFT):
      HAL.setV(0)
      HAL.setW(1)
      state_variable-=1
      if(state_variable<=0):
        state=State.START_BW

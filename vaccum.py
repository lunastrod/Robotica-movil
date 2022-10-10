import random
from HAL import HAL
from GUI import GUI

class State:
    START_SPIRAL=0
    SPIRAL=1
    START_FD=2
    FD=3
    START_BW=4
    BW=5
    START_LEFT=6
    LEFT=7

state=State.START_BW
state_variable=0

while True:
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

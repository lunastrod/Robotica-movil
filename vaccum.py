
import turtle
import random

import turtle
s = turtle.getscreen()
t = turtle.Turtle()
t.speed(0)
#t.penup()

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
      continue
    if(state==State.SPIRAL):
      t.fd(0.1+state_variable/1000)
      t.left(1)
      state_variable+=1
      if(state_variable>=5000):
        state=State.START_SPIRAL
    
    if(state==State.START_BW):
      state_variable=random.randrange(10, 50)
      state=State.BW
    if(state==State.BW):
      t.backward(1)
      state_variable-=1
      if(state_variable<=0):
        state=State.START_LEFT
    if(state==State.START_LEFT):
      state_variable=random.randrange(10, 50)
      state=State.LEFT
    if(state==State.LEFT):
      t.left(1)
      state_variable-=1
      if(state_variable<=0):
        t.fd(10)
        state=State.START_BW

import brickpi3
import time

BP = brickpi3.BrickPi3()
turn_dps = 100
move_dps = -100

def TurnLeft():
      BP.set_motor_dps(BP.PORT_A, turn_dps)
      BP.set_motor_dps(BP.PORT_B, -turn_dps)
      
      time.sleep(1.42)
    
      BP.reset_all()


try:
    for i in range(4):
      BP.set_motor_dps(BP.PORT_A, move_dps)
      BP.set_motor_dps(BP.PORT_B, move_dps)

      time.sleep(4.62)

      TurnLeft() 
    
      BP.reset_all()
except KeyboardInterrupt:
    BP.reset_all()


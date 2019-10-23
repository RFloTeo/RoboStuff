import brickpi3
import time

BP = brickpi3.BrickPi3()

def TurnLeft():
      BP.set_motor_dps(BP.PORT_A, 117.5)
      BP.set_motor_dps(BP.PORT_B, -117.5)
      
      time.sleep(1.9)
    
      BP.reset_all()


try:
    R = 360
    target_angle = R  * 5
    
    for i in range(0, 4):
      BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
      BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))

      BP.set_motor_limits(BP.PORT_A, 30, 0.5 * R)
      BP.set_motor_limits(BP.PORT_B, 30, 0.5 * R)

      print(BP.get_motor_status(BP.PORT_A))

      BP.set_motor_power(BP.PORT_A, 20)
      BP.set_motor_power(BP.PORT_B, 20)

      time.sleep(4.64)

      TurnLeft() 
    
      BP.reset_all()
except KeyboardInterrupt:
    BP.reset_all()


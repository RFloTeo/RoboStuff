import brickpi3
import time

BP = brickpi3.BrickPi3()

try:
  for i in range(0, 4):
      BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
      BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
      BP.set_motor_power(BP.PORT_A, 30)
      BP.set_motor_power(BP.PORT_B, 30)
      time.sleep(2.95)
      BP.set_motor_power(BP.PORT_A, 30)
      BP.set_motor_power(BP.PORT_B, -30)
      time.sleep(1.45)
      BP.reset_all()
      time.sleep(0.5)
except KeyboardInterrupt:
    BP.reset_all()

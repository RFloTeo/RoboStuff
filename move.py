import brickpi3
import time

BP = brickpi3.BrickPi3()

BP.set_motor_power(BP.PORT_A, 50)
BP.set_motor_power(BP.PORT_B, 50)
time.sleep(5)
BP.reset_all()

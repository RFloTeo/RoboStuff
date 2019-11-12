import brickpi3
import time

BP = brickpi3.BrickPi3()
dps = 100

def turn360():
    BP.set_motor_dps(BP.PORT_A, dps)
    BP.set_motor_dps(BP.PORT_B, -dps)

    time.sleep(5.45)

    BP.reset_all()


turn360()

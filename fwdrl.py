import brickpi3
import time

BP = brickpi3.BrickPi3()

RMotor = BP.PORT_B
LMotor = BP.PORT_C

SPEED = 140

BP.set_motor_limits(RMotor, 70, SPEED)
BP.set_motor_limits(LMotor, 70, SPEED)

def forward():
    BP.offset_motor_encoder(RMotor, BP.get_motor_encoder(RMotor))
    BP.offset_motor_encoder(LMotor, BP.get_motor_encoder(LMotor))
    BP.set_motor_position(RMotor, -SPEED)
    BP.set_motor_position(LMotor, -SPEED)
    time.sleep(1)

def turn_right_90():
    BP.offset_motor_encoder(RMotor, BP.get_motor_encoder(RMotor))
    BP.offset_motor_encoder(LMotor, BP.get_motor_encoder(LMotor))
    BP.set_motor_position(RMotor, -SPEED)
    BP.set_motor_position(LMotor, SPEED)
    time.sleep(1)

def turn_left_90():
    BP.offset_motor_encoder(RMotor, BP.get_motor_encoder(RMotor))
    BP.offset_motor_encoder(LMotor, BP.get_motor_encoder(LMotor))
    BP.set_motor_position(RMotor, SPEED)
    BP.set_motor_position(LMotor, -SPEED)
    time.sleep(1)

try:
    for i in range(4):
       forward()   
       forward()
       forward()
       forward()
       turn_left_90()
    BP.reset_all()
except KeyboardInterrupt:
    BP.reset_all()


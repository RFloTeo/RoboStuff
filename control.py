#!/usr/bin/env python3


import brickpi333 as brickpi3 


BP = brickpi3.BrickPi333()
def reset():
  BP.reset_all()
  BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
  BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
  BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.CUSTOM, [(BP.SENSOR_CUSTOM.PIN1_ADC)]) 
  BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.NXT_ULTRASONIC)
  BP.set_motor_limits(BP.PORT_A, 30, 0.5 * 360)
  BP.set_motor_limits(BP.PORT_B, 30, 0.5 * 360)

import sys,tty,os,termios
def getkey():
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            b = os.read(sys.stdin.fileno(), 3).decode()
            if len(b) == 3:
                k = ord(b[2])
            elif len(b) == 1:
                k = ord(b)
            else:
                k = ord(b[0])
            key_mapping = {
                127: 'backspace',
                10: 'return',
                32: 'space',
                9: 'tab',
                27: 'esc',
                65: 'up',
                66: 'down',
                67: 'right',
                68: 'left'
            }
            return key_mapping.get(k, chr(k))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

try:
    reset()
    print("Started")
    while True:
        k = getkey()
        if k == 'esc':
          quit()
        elif k == 'up' or k == 'w':
          BP.set_motor_dps(BP.PORT_A, -140)
          BP.set_motor_dps(BP.PORT_B, -140)
        elif k == 'down' or k == 's':
          BP.set_motor_dps(BP.PORT_A, 140)
          BP.set_motor_dps(BP.PORT_B, 140)
        elif k == 'left' or k == 'a':
          BP.set_motor_dps(BP.PORT_A, 80)
          BP.set_motor_dps(BP.PORT_B, -80)
        elif k == 'right' or k == 'd':
          BP.set_motor_dps(BP.PORT_A, -80)
          BP.set_motor_dps(BP.PORT_B, 80)
        elif k == 'z':
          reading = []
          while len(reading) < 10:
            try:
                tryread = BP.get_sensor(BP.PORT_2)
                if tryread != 255 and tryread > 10:
                    reading.append(tryread)
            except:
                pass
          mean = sum(reading) / len(reading)
          print("I measured: ", mean)
        elif k == 'space':
          reset()
        else:
          print(k)
except (KeyboardInterrupt, SystemExit):
    reset()
    print('stopping.')
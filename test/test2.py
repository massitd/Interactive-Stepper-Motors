import board 
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit

kit = MotorKit(i2c=board.I2C())

for i in range(10000):
    kit.stepper1.onestep(style=stepper.MICROSTEP)

kit.stepper1.release()
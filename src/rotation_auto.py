from vex import *

GEAR_RATIO = 0.67
WHEEL_CIRC = 12.57

def motor_rot_avg():
    # Get the average rotation of each motor, to get a 
    # More accurate reading of far much we have gone
    num = 0

    for motor in motors:
        num += motor.position()

    return num / len(motors)

def driven_dist(self):
    # Get the distance we have driven based on rotation
    return (motor_rot_avg() / 360) * GEAR_RATIO * WHEEL_CIRC

def drive_for(distance: float):
    while motor_rot_avg() < distance:
        drivetrain.drive_for(FORWARD, 50, PERCENT):

    drivetrain.stop(BRAKE)

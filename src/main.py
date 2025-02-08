from vex import *
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
mgR_top = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
mgR_bottom = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
mgR = MotorGroup(mgR_top, mgR_bottom)
mgL_top = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False)
mgL_bottom = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
mgL = MotorGroup(mgL_top, mgL_bottom)
controller = Controller(PRIMARY)
inertial = Inertial(Ports.PORT18)
drivetrain = SmartDrive(mgL, mgR, inertial, 319.19, 330, 320, MM, 1)
lift = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)
clamp = DigitalOut(brain.three_wire_port.a)
optical = Optical(Ports.PORT5)

motors = [mgL_top, mgL_bottom, mgR_top, mgR_bottom]

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

# define variables used for controlling motors based on controller inputs
lift_stopped = True
lift_toggled = False
drive_l_must_stop = False
drive_r_must_stop = False

#Event to be called when button is pressed: Toggles the digital out for the Pneumatic clamp
def toggle_clamp():
    clamp.set(not clamp.value())

def toggle_lift():
    global lift_toggled
    lift_toggled = not lift_toggled

# Register the functions to toggle components
controller.buttonDown.pressed(toggle_clamp) 
controller.buttonRight.pressed(toggle_lift)

# Configure the lift
lift.set_velocity(100, PERCENT)
lift.set_stopping(COAST)

# Constants are a type of variable that don't change, or are always constant
# It's easier to know what's going on when there are words rather than the same
# numbre all over the place.
WHEEL_BASE = 13.5 # Inches
WHEEL_DIAMETER = 4 # Inches

MAX_TURN_SPEED = 55 # Percent
MAX_SPEED = 100 # Percent
ACCELERATION = 85 # Percent per second 
GEAR_RATIO = 2 / 3 # Gear Ratio of the drivetrain
WHEEL_CIRC = WHEEL_DIAMETER * math.pi # Circumference of the omni wheels

# define a task that will handle monitoring inputs from controller_1
def remote_control_loop():
    global drive_l_must_stop, drive_r_must_stop, lift_stopped, lift_toggled, remote_control_code_enabled
    global last_time, delta, velocity_left, velocity_right
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    last_time = brain.timer.time(SECONDS)
    delta = 0
    velocity_left = 0
    velocity_right = 0

    while True:
        delta = brain.timer.time(SECONDS) - last_time
        last_time = brain.timer.time(SECONDS)

        if remote_control_code_enabled:
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis1
            # right = axis3 - axis1
            drivetrain_left_axis_value = controller.axis3.position() + controller.axis1.position()
            drivetrain_right_axis_value = controller.axis3.position() - controller.axis1.position()

            velocity_left += ACCELERATION * delta
            velocity_right += ACCELERATION * delta

            drivetrain_right_axis_value = min(abs(drivetrain_right_axis_value), velocity_right) * (-1 if drivetrain_right_axis_value < 0 else 1)
            drivetrain_left_axis_value = min(abs(drivetrain_left_axis_value), velocity_left) * (-1 if drivetrain_left_axis_value < 0 else 1)

            # check if the value is inside of the deadband range
            if drivetrain_left_axis_value < 5 and drivetrain_left_axis_value > -5:
                # check if the left motor has already been stopped
                if drive_l_must_stop:
                    velocity_left = 0
                    # stop the left drive motor
                    mgL.stop()
                    # tell the code that the left motor has been stopped
                    drive_l_must_stop = False
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                drive_l_must_stop = True
            # check if the value is inside of the deadband range
            if drivetrain_right_axis_value < 5 and drivetrain_right_axis_value > -5:
                # check if the right motor has already been stopped
                if drive_r_must_stop:
                    velocity_right = 0
                    # stop the right drive motor
                    mgR.stop()
                    # tell the code that the right motor has been stopped
                    drive_r_must_stop = False
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                drive_r_must_stop = True

            # only tell the left drive motor to spin if the values are not in the deadband range
            if drive_l_must_stop:
                mgL.set_velocity(drivetrain_left_axis_value, PERCENT)
                mgL.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drive_r_must_stop:
                mgR.set_velocity(drivetrain_right_axis_value, PERCENT)
                mgR.spin(FORWARD)

            # When the lift is toggled via the right button with toggle_lift,
            # the lift will constantly go. If the lift is manually triggered to 
            # go forward or backward, it will untoggle the lift and switch back
            # to manual mode. 
            if controller.buttonY.pressing():
                lift.spin(FORWARD)
                lift_stopped = False
                lift_toggled = False
            elif controller.buttonB.pressing():
                lift.spin(REVERSE)
                lift_stopped = False
                lift_toggled = False
            elif lift_toggled:
                lift.spin(FORWARD)
                lift_stopped = False
            elif not lift_stopped:
                lift.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                lift_stopped = True

        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True
remote_control_thread = Thread(remote_control_loop)

def driver_control():
    temp_thread = Thread(screen)
    print("Control driver")

def motor_rot_avg():
    # Get the average rotation of each motor, to get a 
    # More accurate reading of far much we have gone
    num = 0

    for motor in motors:
        num += abs(motor.position())

    return num / len(motors)

def reset_pos():
    for motor in motors:
        motor.reset_position()

def driven_dist():
    # Get the distance we have driven based on rotation
    dist = (motor_rot_avg() / 360) * GEAR_RATIO * WHEEL_CIRC
    return dist

def threaded_spin(motor, *args):
    Thread(motor.spin_for, args)

class Auto:
    """
    This auto class is used for organization purposes. It makes sure programmers don't use 
    the drive_for and turn_for auto functions unless they know what they're doing.
    """
    @staticmethod
    def drive_for_auto(direction, distance_in: float, velocity_percent: int, stop_type = BRAKE):
        reset_pos()
        wait(25, MSEC)
        while driven_dist() < distance_in:
            drivetrain.drive(direction, velocity_percent, PERCENT)
        
        drivetrain.stop(stop_type)

    @staticmethod
    def turn_for_auto(direction, distance_deg: int, velocity_percent: int):
        reset_pos()

        turn_radius = WHEEL_BASE / 2
        turning_circumference = 2 * math.pi * turn_radius
        turn_dist = (distance_deg / 360) * turning_circumference # The amount of inches the wheels need to travel to rotate that amount of degrees

        while driven_dist() < turn_dist:
            drivetrain.turn(direction, velocity_percent, PERCENT)

        drivetrain.stop(BRAKE)

    def left_auto(self):
        # Assuming the robot starts as far behind the line as possible
        # lift.spin_for(FORWARD, 500, MSEC) # intake the preload

        threaded_spin(lift, 1500, MSEC)
        self.drive_for_auto(FORWARD, 36, 50) # Go 40 inches towards the rings
        wait(250, MSEC)
        # lift.spin_for(FORWARD, 1, SECONDS) # Collect the ring while we're moving

        self.turn_for_auto(LEFT, 80, 10) # Turn 90 degrees to have the rear face a goal
        wait(250, MSEC)

        self.drive_for_auto(REVERSE, 16, 10, COAST) # Reverse into the mobile goal
        toggle_clamp() # Hopefully clamp the mobile goal
        self.drive_for_auto(REVERSE, 6, 10, COAST) # Hopefully this makes it keep going 

        lift.spin_for(FORWARD, 5, SECONDS) # Score the rings while we're moving

        # self.drive_for_auto(FORWARD, 16, 50)

    def dummy_auto(self):
        # Just drive off the line. 
        self.drive_for_auto(FORWARD, 6, 50)

def screen():
    # Have the controller show the temperatures of the drivetrain motors, for some reason.
    scr = controller.screen
    while True:
        # Gather all of the temperatures
        RT_temp = mgR_top.temperature(TemperatureUnits.FAHRENHEIT)
        RB_temp = mgR_bottom.temperature(TemperatureUnits.FAHRENHEIT)
        LT_temp = mgL_top.temperature(TemperatureUnits.FAHRENHEIT)
        LB_temp = mgL_bottom.temperature(TemperatureUnits.FAHRENHEIT)

        scr.clear_screen() # Make sure we aren't writing over anything

        # Write all of the temperature data to the screen.
        # I have no idea how many columns the controller has, so this may go off the
        # screen or be horribly uncentered. This uses about 20 characters per row.
        scr.set_cursor(1, 1)
        scr.print("LT - " + str(int(LT_temp)))
        scr.set_cursor(1, 10)
        scr.print(" | RT - " + str(int(RT_temp)))
        scr.set_cursor(2, 1)
        scr.print("LB - " + str(int(LB_temp)))
        scr.set_cursor(2, 10)
        scr.print(" | RB - " + str(int(RB_temp)))

        wait(5, SECONDS) # Update the motor temperatures on the screen only every five seconds

clamp.set(True) # So the clamp is up when the game starts
comp = Competition(driver_control, Auto().dummy_auto)
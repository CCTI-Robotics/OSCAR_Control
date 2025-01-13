#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
mgR_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
mgR_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
mgR = MotorGroup(mgR_motor_a, mgR_motor_b)
mgL_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False)
mgL_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
mgL = MotorGroup(mgL_motor_a, mgL_motor_b)
controller_1 = Controller(PRIMARY)
left_drive_smart = mgL
right_drive_smart = mgR
drive_inertial = Inertial(Ports.PORT15)
drivetrain = SmartDrive(left_drive_smart, right_drive_smart, drive_inertial, 319.19, 330, 320, MM, 1)
lift = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)
pneum_clamp = DigitalOut(brain.three_wire_port.a)
opt_rear = Optical(Ports.PORT5)
rotation = Rotation(Ports.PORT18)

# wait for rotation sensor to fully initialize
wait(30, MSEC)

# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()

def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

# define variables used for controlling motors based on controller inputs
controller_1_right_scuff_control_motors_stopped = True
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

#Event to be called when button is pressed: Toggles the digital out for the Pneumatic clamp
def toggle_clamp():
    if pneum_clamp.value():
        pneum_clamp.set(False)
    else:
        pneum_clamp.set(True)

#Operates the Pneumatic clamp
controller_1.buttonDown.pressed(toggle_clamp)

MAX_TURN_SPEED = 55 # Percent
MAX_SPEED = 100 # Percent
ACCELERATION = 85 # Percent per second 

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, controller_1_right_scuff_control_motors_stopped, remote_control_code_enabled
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
            drivetrain_left_axis_value = controller_1.axis3.position() + controller_1.axis1.position()
            drivetrain_right_axis_value = controller_1.axis3.position() - controller_1.axis1.position()

            velocity_left += ACCELERATION * delta
            velocity_right += ACCELERATION * delta

            drivetrain_right_axis_value = min(abs(drivetrain_right_axis_value), velocity_right) * (-1 if drivetrain_right_axis_value < 0 else 1)
            drivetrain_left_axis_value = min(abs(drivetrain_left_axis_value), velocity_left) * (-1 if drivetrain_left_axis_value < 0 else 1)

            # check if the value is inside of the deadband range
            if drivetrain_left_axis_value < 5 and drivetrain_left_axis_value > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    velocity_left = 0
                    # stop the left drive motor
                    left_drive_smart.stop()
                    # tell the code that the left motor has been stopped
                    drivetrain_l_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                drivetrain_l_needs_to_be_stopped_controller_1 = True
            # check if the value is inside of the deadband range
            if drivetrain_right_axis_value < 5 and drivetrain_right_axis_value > -5:
                # check if the right motor has already been stopped
                if drivetrain_r_needs_to_be_stopped_controller_1:
                    velocity_right = 0
                    # stop the right drive motor
                    right_drive_smart.stop()
                    # tell the code that the right motor has been stopped
                    drivetrain_r_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                drivetrain_r_needs_to_be_stopped_controller_1 = True

            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_l_needs_to_be_stopped_controller_1:
                left_drive_smart.set_velocity(drivetrain_left_axis_value, PERCENT)
                left_drive_smart.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_r_needs_to_be_stopped_controller_1:
                right_drive_smart.set_velocity(drivetrain_right_axis_value, PERCENT)
                right_drive_smart.spin(FORWARD)
            # check the buttonR1/buttonR2 status
            # to control lift
            if controller_1.buttonY.pressing():
                lift.spin(FORWARD, 100, PERCENT)
                controller_1_right_scuff_control_motors_stopped = False
            elif controller_1.buttonB.pressing():
                lift.spin(REVERSE, 100, PERCENT)
                controller_1_right_scuff_control_motors_stopped = False
            elif not controller_1_right_scuff_control_motors_stopped:
                lift.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_right_scuff_control_motors_stopped = True


        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True

# rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

def driver_control():
    print("Control driver")

def no_auto():
    ... # Skip the autonomous period.

def min_auto():
    # Auto that only grabs a goal. 
    # min_auto works in any quadrant. 

    while not opt_rear.is_near_object():
        drivetrain.drive(REVERSE, 45, PERCENT)

    pneum_clamp.set(False)
    wait(250, MSEC)
    drivetrain.stop(BRAKE)

def right_auto():
    while drive_inertial.is_calibrating():
        wait(100, MSEC)

    min_auto() # Grab the goal 

    drivetrain.turn_for(LEFT, 80, DEGREES, 10, PERCENT)
    lift.spin(FORWARD, 75, PERCENT)
    drivetrain.drive_for(FORWARD, 35, INCHES, 25, PERCENT)
    drivetrain.turn_for(RIGHT, 100, DEGREES,10, PERCENT)
    drivetrain.drive_for(FORWARD, 12, INCHES, 100, PERCENT)
    drivetrain.drive_for(REVERSE, 12, INCHES, 50, PERCENT)
    drivetrain.turn_for(RIGHT, 42, DEGREES)
    #drivetrain.drive_for(FORWARD, 78, INCHES, 50, PERCENT)

def left_auto():
    while drive_inertial.is_calibrating():
        wait(100, MSEC)

    min_auto() # Grab the goal 

    drivetrain.turn_for(RIGHT, 80, DEGREES, 10, PERCENT)
    lift.spin(FORWARD, 75, PERCENT)
    drivetrain.drive_for(FORWARD, 35, INCHES, 25, PERCENT)
    drivetrain.turn_for(LEFT, 100, DEGREES,10, PERCENT)
    drivetrain.drive_for(FORWARD, 12, INCHES, 100, PERCENT)
    drivetrain.drive_for(REVERSE, 12, INCHES, 50, PERCENT)
    drivetrain.turn_for(RIGHT, 42, DEGREES)
    #drivetrain.drive_for(FORWARD, 78, INCHES, 50, PERCENT)

# Experimental Auto Selector

# def auto_select():
#     scr = controller_1.screen
#     scr.clear_screen()
    
#     scr.print("A: Right Side")
#     scr.print("X: Left Side")
#     scr.print("UP: Min Auto")

# is_left_auto = False
# is_right_auto = False
# is_min_auto = False

# def select_left():
#     global is_left_auto
#     is_left_auto = True
#     print("Left")

# def select_right():
#     global is_right_auto
#     is_right_auto = True
#     print("Right")

# def select_min():
#     global is_min_auto
#     is_min_auto = True
#     print("Min")

# def auto():
#     for config in [(is_min_auto, min_auto), (is_left_auto, left_auto), (is_right_auto, right_auto)]:
#         if config[0]:
#             config[1]()
#             break 

# controller_1.buttonA.pressed(select_right)
# controller_1.buttonX.pressed(select_left)
# controller_1.buttonUp.pressed(select_min)

def screen():
    # Have the controller show the temperatures of the drivetrain motors, for some reason.
    scr = controller_1.screen
    while True:
        # Gather all of the temperatures
        RT_temp = mgR_motor_a.temperature(TemperatureUnits.FAHRENHEIT)
        RB_temp = mgR_motor_b.temperature(TemperatureUnits.FAHRENHEIT)
        LT_temp = mgL_motor_a.temperature(TemperatureUnits.FAHRENHEIT)
        LB_temp = mgL_motor_b.temperature(TemperatureUnits.FAHRENHEIT)

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

        wait(2, SECONDS) # Update the motor temperatures on the screen only every five seconds


pneum_clamp.set(True)
drive_inertial.calibrate()

temp_thread = Thread(screen)

rotation.reset_position()

while True:
    print("Rotation sensor: ", rotation.position())
    print("Motor: ", mgL_motor_b.position())

    wait(3, SECONDS)


comp = Competition(driver_control, min_auto)
#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
mgR_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
mgR_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
mgR = MotorGroup(mgR_motor_a, mgR_motor_b)
mgL_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False)
mgL_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
mgL = MotorGroup(mgL_motor_a, mgL_motor_b)
controller_1 = Controller(PRIMARY)
left_drive_smart = mgL
right_drive_smart = mgR
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
lift = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)
pneum_clamp = DigitalOut(brain.three_wire_port.a)
distance_rear = Distance(Ports.PORT5)
distance_front = Distance(Ports.PORT6)

MAX_TURN_SPEED = 5

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
    if pneum_clamp.value:
        pneum_clamp.set(False)
    else:
        pneum_clamp.set(True)

#Operates the Pneumatic clamp
controller_1.buttonY.pressed(toggle_clamp)

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, controller_1_right_scuff_control_motors_stopped, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis1
            # right = axis3 - axis1
            drivetrain_left_side_speed = controller_1.axis3.position() + controller_1.axis1.position()
            drivetrain_right_side_speed = controller_1.axis3.position() - controller_1.axis1.position()
            
            # # USER-DEFINED
            # # Limit the speed of both motors if the motors are turning in opposite directions (or the robot is rotating)
            # if (drivetrain_left_side_speed > 0 and drivetrain_right_side_speed < 0) or (drivetrain_left_side_speed < 0 and drivetrain_right_side_speed > 0):
            #     drivetrain_left = min(abs(drivetrain_left_side_speed), MAX_TURN_SPEED)
            #     drivetrain_right = min(abs(drivetrain_right_side_speed), MAX_TURN_SPEED)

            #     # Set the motors to the new values
            #     drivetrain_left_side_speed = drivetrain_left if drivetrain_left_side_speed > 0 else -drivetrain_left
            #     drivetrain_right_side_speed = drivetrain_right if drivetrain_right_side_speed > 0 else -drivetrain_right

            # check if the value is inside of the deadband range
            if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    # stop the left drive motor
                    left_drive_smart.stop()
                    # tell the code that the left motor has been stopped
                    drivetrain_l_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                drivetrain_l_needs_to_be_stopped_controller_1 = True
            # check if the value is inside of the deadband range
            if drivetrain_right_side_speed < 5 and drivetrain_right_side_speed > -5:
                # check if the right motor has already been stopped
                if drivetrain_r_needs_to_be_stopped_controller_1:
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
                left_drive_smart.set_velocity(drivetrain_left_side_speed, PERCENT)
                left_drive_smart.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_r_needs_to_be_stopped_controller_1:
                right_drive_smart.set_velocity(drivetrain_right_side_speed, PERCENT)
                right_drive_smart.spin(FORWARD)
            # check the buttonR1/buttonR2 status
            # to control lift
            if controller_1.buttonA.pressing():
                lift.spin(FORWARD)
                controller_1_right_scuff_control_motors_stopped = False
            elif controller_1.buttonB.pressing():
                lift.spin(REVERSE)
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

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)


def driver_control():
    print("Control driver")

def autonomous():
    controller_1.screen.print("Auto Start!")
    
    MAX_GOAL_DIST_MM = 145

    # Expecting the robot to start facing backwards, the drivetrain will
    # reverse until the distance sensor notices an object (a mobile goal)
    while distance_rear.object_distance() > MAX_GOAL_DIST_MM:
        drivetrain.drive(REVERSE)
        time.sleep(0.1)  # To not overwork the distance sensor

    # Stop moving the robot and clamp the mobile goal 
    drivetrain.stop()
    pneum_clamp.set(True)

    # Spin around until a stack of rings are located. Then, advance to them
    while distance_front.object_distance() > 300:
        drivetrain.turn(RIGHT)
        time.sleep(0.1)  # To not overwork the distance sensor

    drivetrain.drive(FORWARD)
    lift.spin(FORWARD)  # Turn the lift and intake to get ready to score some rings

    





comp = Competition(driver_control, autonomous)
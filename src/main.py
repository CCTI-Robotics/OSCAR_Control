from vex import *
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
mgR_top = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)
mgR_bottom = Motor(Ports.PORT12, GearSetting.RATIO_6_1, False)
mgR = MotorGroup(mgR_top, mgR_bottom)
mgL_top = Motor(Ports.PORT16, GearSetting.RATIO_6_1, False)
mgL_bottom = Motor(Ports.PORT17, GearSetting.RATIO_6_1, True)
mgL = MotorGroup(mgL_top, mgL_bottom)
controller = Controller(PRIMARY)
# inertial = Inertial(Ports.PORT18)
drivetrain = DriveTrain(mgL, mgR, 319.19, 330, 320, MM, 1)
lift = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
roller = Motor(Ports.PORT10, True)
intake = MotorGroup(lift, roller)
clamp = DigitalOut(brain.three_wire_port.a)
pneum_lift_1 = DigitalOut(brain.three_wire_port.g)
pneum_lift_2 = DigitalOut(brain.three_wire_port.h)
optical = Optical(Ports.PORT6)

motors = [mgL_top, mgL_bottom, mgR_top, mgR_bottom]

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

# define variables used for controlling motors based on controller inputs
intake_stopped = True
intake_toggled = False
drive_l_must_stop = False
drive_r_must_stop = False

def toggle_clamp():
    """
    An event to be called when the Down button is pressed
    to toggle the pneumatic clamp.
    """
    clamp.set(not clamp.value())

def toggle_intake():
    """
    An event to be called when the Right button is pressed to 
    toggle the lift. The lift will be automatically disabled
    when a manual control is pressed. See remote_control_loop()
    """
    global intake_toggled
    intake_toggled = not intake_toggled


def toggle_hang():
    """
    Extend the arms that allow the robot to hang in the air.
    I decided to use pneum_lift_1's value for both so in case
    they have a different value for any reason, they sync back up
    once hanging is toggled.
    """
    pneum_lift_1.set(not pneum_lift_1.value())
    pneum_lift_2.set(not pneum_lift_1.value())


# Register the events to be pressed
controller.buttonDown.pressed(toggle_clamp) 
controller.buttonRight.pressed(toggle_intake)
controller.buttonR2.pressed(toggle_hang)

# Configure the lift
intake.set_velocity(100, PERCENT)
intake.set_stopping(COAST) # I'm not sure if this actually changes anything

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

def remote_control_loop():
    """
    Monitors inputs from the controller in order to control the robot. 
    This is mostly for the drivetrain, since other controls are handled by on_pressed
    """
    # Everything is global so they are set outside of the current function instead of just being 
    # redefined here
    global drive_l_must_stop, drive_r_must_stop, intake_stopped, intake_toggled, remote_control_code_enabled
    global last_time, delta, velocity_left, velocity_right

    # Define variables to be used only in this method
    last_time = brain.timer.time(SECONDS) # The last time of the loop to calculate delta
    delta = 0 # Time since the last loop
    # Velocities, in percent, for each side of the drivetrain
    velocity_left = 0
    velocity_right = 0

    # Usually, a `while True` loop would block anything else from happening. Fortunately, this
    # method is supposed to run in a thread, which means it runs in `parallel` aside the main process
    while True:
        delta = brain.timer.time(SECONDS) - last_time # Delta is current_time - last_time
        last_time = brain.timer.time(SECONDS) # Set last time again

        if remote_control_code_enabled:
            # Calculate the drivetrain motor velocities from the controller joystick axes
            # left = axis3 + axis1
            # right = axis3 - axis1
            drivetrain_left_axis_value = controller.axis3.position() + controller.axis1.position()
            drivetrain_right_axis_value = controller.axis3.position() - controller.axis1.position()

            # The calculated velocity is the acceleration (%/s) times delta
            # This means, since %/s is percent per second, a delta of one second
            # would accelerate the drivetrain %. This makes sure acceleration is
            # constant, so if delta is less than a second it still accelerates 
            # at the same rate. 
            velocity_left += ACCELERATION * delta
            velocity_right += ACCELERATION * delta

            # This uses a lot of inline methods and conditions. Let's break it down.
            # min() is a method that takes the smaller of x values. min(1, 2) = 1
            # abs() is the absolute value of a number. abs(-1) = 1
            # `x if z else y` is an inline condition, which is equivalent to a function containing the following:
            # if z:
            #     return x
            # else:
            #     return y
            # We use abs() on the controller axis value because we don't care about the direction of the force,
            # just the set velocity. We then take what's smaller, the velocity the robot wants to go, or the velocity
            # calculated by the script (using our acceleration). We take what's smaller since, if the driver has the stick lower,
            # it means they don't want the robot to go as fast. If the acceleration is lower, it means the robot is still accelerating
            # and has not reached max speed, so we should not let the driver tell it to go faster. 
            # We then multiply this minimum value by -1 if the original axis value was negative, to get the force in the proper direction
            # again. 
            #
            # TL:DR; Make sure the robot does not go faster than the calculated velocity and desired velocity
            drivetrain_right_axis_value = min(abs(drivetrain_right_axis_value), velocity_right) * (-1 if drivetrain_right_axis_value < 0 else 1)
            drivetrain_left_axis_value = min(abs(drivetrain_left_axis_value), velocity_left) * (-1 if drivetrain_left_axis_value < 0 else 1)

            # Check if left stick is in deadband range.
            # Being in deadband range happens when the stick is let go of. We ignore values
            # between 5 and -5 because, when the stick is let go of, it's not always going to return 0.
            if drivetrain_left_axis_value < 5 and drivetrain_left_axis_value > -5:
                # Check to see if the left motor is stopped already. The variable name 
                # `drive_l_must_stop` means the motor is not yet stopped, but it should be the next time
                # this variable is checked. We do it this way because constantly telling the motor to stop
                # when we're within deadband is taxing to the motor and adds unnecessary overhead to the code,
                # making it slower and using more resources. 
                if drive_l_must_stop:
                    velocity_left = 0
                    # stop the left drive motor
                    mgL.stop()
                    # tell the code that the left motor has been stopped
                    drive_l_must_stop = False
            else:
                # Since the robot is moving, we reset the variable so it can be changed next time the
                # stick is let go of. Even though this variable is True, it is never used to stop the 
                # motor until the stick is in the deadband range again. 
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

            # If the motors *must* stop, it means they haven't yet stopped. Therefore, we
            # apply a new velocity to the motor and make them go forward.
            if drive_l_must_stop:
                mgL.set_velocity(drivetrain_left_axis_value, PERCENT)
                mgL.spin(FORWARD)
                
            if drive_r_must_stop:
                mgR.set_velocity(drivetrain_right_axis_value, PERCENT)
                mgR.spin(FORWARD)

            # When the lift is toggled via the right button with toggle_lift,
            # the lift will constantly go. If the lift is manually triggered to 
            # go forward or backward, it will untoggle the lift and switch back
            # to manual mode. 
            if controller.buttonY.pressing():
                intake.spin(FORWARD)
                intake_stopped = False # Since the lift is spinning, it is not stopped
                intake_toggled = False # The lift should no longer be toggled since we switched to manual mode
                
            elif controller.buttonB.pressing():
                intake.spin(REVERSE)
                intake_stopped = False
                intake_toggled = False
                
            elif intake_toggled:
                intake.spin(FORWARD)
                intake_stopped = False
                
            elif not intake_stopped: # If none of the before conditions are true, and the lift is not stopped, stop it. 
                intake.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                intake_stopped = True

        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True
remote_control_thread = Thread(remote_control_loop)

def motor_rot_avg():
    """
    Take the rotation of each motor to return an average, to get a more accurate representation
    of how far the robot has driven. 
    """
    num = 0

    for motor in motors:
        num += abs(motor.position()) # Get the absolute value to tell us how far it's driven, doesn't matter what direction

    return num / len(motors)

def reset_pos():
    """
    Reset the position of each motor to prepare them to measure another distance
    """
    for motor in motors:
        motor.reset_position()

def driven_dist():
    """
    Calculate the distance the robot has driven using the rotation of the wheels along with the 
    gear ratio and wheel circumeference. 
    """
    dist = (motor_rot_avg() / 360) * GEAR_RATIO * WHEEL_CIRC
    return dist

def threaded_spin(motor: "Motor | MotorGroup", *args):
    """
    Make a motor spin in a thread so it's not blocking.
    Useful for auto when we need to spin a motor like the intake but also do other stuff
    at the same time. 
    """
    Thread(motor.spin_for, args)

class Auto:
    """
    This auto class is used for organization purposes. It makes sure programmers don't use 
    the drive_for and turn_for auto functions unless they know what they're doing.
    """
    def __init__(self):
        # You'll take a look at the available autos and see all of these lambdas. What's a lambda?
        # `lambda` is a keyword that's used to define an inline function. A lambda that's defined as 
        # `lambda a: a + 1` is equivalent to the following code:
        # def f(a):
        #     return a + 1
        # Lambda is useful so we can make an anonymous function that we only use once for a small amount of code
        # Since the Competition() class doesn't accept arguments for the methods, if we *do* need to pass an 
        # argument for the autos, like in auto_minus, we use lambda to make a single-use function that calls
        # the method with the required arguments. So, when you see the line `lambda: self.auto_minus(Color.Red)`,
        # that's the same as a function:
        # def f():
        #     return self.auto_minus(Color.Red)
        #
        # You'll also notice the odd syntax of (x, y). This is a `tuple`. Similar to a list, a tuple is an 
        # immutable collection of objects. In this case, for each auto, we create a tuple where index 0 is
        # the name of the auto, and index 1 is the function that calls the auto. This makes it easier to manage
        # in the selector. These tuples are then stored in a list, so if we wanted to retrieve an item from a 
        # tuple, it would be list[index_of_list][index_of_tuple], since list[index] returns the item of the list,
        # in this case a tuple, which we can then immediately use by indexing the tuple. Confusing, I know. 
        self.available_autos = [
            ("Red Minus", lambda: self.auto_minus(Color.RED)),
            ("Blue Minus", lambda: self.auto_minus(Color.BLUE)),
            ("Min", lambda: self.auto_min()),
            ("Min + Score", lambda: self.auto_direct_score()),
            ("Blank", lambda: ...) # ... is the same as `pass`. This is an empty function
        ] # A list of tuples that contain a name for the auto and the method itself
        self.selected_auto = 2 # Index of available_autos
        self.roller_should_be_moving = False
        self.lift_should_be_moving = True

    @staticmethod
    def drive_for_auto(direction, distance_in: float, velocity_percent: int, stop_type = BRAKE):
        """
        Drive a certain distance. This algorithm uses the amount of rotations the motors
        to check if the bot has driven a certain distance.
        """
        reset_pos() # Reset position so we don't count previously driven distance in our new calculation
        wait(25, MSEC) # Wait a small time for the positions to update
        
        while driven_dist() < distance_in: # Drive until we reach our destination
            drivetrain.drive(direction, velocity_percent, PERCENT)
        
        drivetrain.stop(stop_type)

    @staticmethod
    def turn_for_auto(direction, distance_deg: int, velocity_percent: int):
        """
        Autonomously turn a certain amount. This algorithm uses the rotations of the
        motor to dictate how far the robot has rotated. Seems to be decently innacurate.
        """
        reset_pos() # Make sure the position readings are fresh

        turn_radius = WHEEL_BASE / 2 
        turning_circumference = 2 * math.pi * turn_radius
        turn_dist = (distance_deg / 360) * turning_circumference # The amount of inches the wheels need to travel to rotate that amount of degrees

        while driven_dist() < turn_dist:
            drivetrain.turn(direction, velocity_percent, PERCENT)

        drivetrain.stop(BRAKE)

    def auto_min(self):
        """
        Description:
        min_auto only goes forward until it grabs a goal.
        Useful to get off of the starting line
        This is also useful in regular auto processes so we don't have to 
        repeat the same code. Reusability!

        Criteria:
        - Any side of field
        - Must have a direct, straight path to a goal
        - Must be facing backwards (intake towards wall)
        """
        reset_pos()

        # Prepare the optical sensor
        optical.set_light(100)
        # "Object Detect Threshold" isn't defined anywhere to tell us what exactly
        # that means. I've learned that it only goes up to 255 and the higher the 
        # number, the closer an object needs to be to be detected. If the value is 
        # 255, it just never detects anything. We want to get as close as possible
        # without being detected to be more accurate.
        optical.object_detect_threshold(254)
        
        # Drive until the optical sensor on the back of the robot detects
        # a goal or until we have driven 48 inches.
        # This is a failsafe in case the sensor or process malfunctions
        # to make sure we don't go over the line and get DQ'd
        timer = False

        while driven_dist() < 48:
            drivetrain.drive(REVERSE, 45, PERCENT)

            if timer and timer.time() >= 500:
                clamp.set(True)
                break

            if optical.is_near_object() and not timer:
                timer = Timer()
            
            wait(5, MSEC)

        # Set the clamp down to make sure we grab the goal. We set it to False manually instead
        # of using the toggle method to ensure the clamp is down instead of putting it back
        # up if it was already down (which shouldn't normally happen).
        clamp.set(True)
        wait(250, MSEC)
        drivetrain.stop(BRAKE)

    def auto_minus(self, color):
        """
        An auto that works on the minus side of the field. This 
        auto will, with a preload, grab another ring (be in possession of two), get a 
        mobile goal, and (attempt) to score both rings on the mobile goal.

        Criteria:
        - Left side of either team
        - Must be facing a double stack of rings
            - would theoretically work with a single ring as well
        - Must be facing forward (clamp toward wall)
        """

        threaded_spin(intake, 1500, MSEC) # Spin the intake while we move toward the field ring
        self.drive_for_auto(FORWARD, 36, 50) # Go 40 inches towards the rings
        wait(250, MSEC) # Wait a small amount of time to make sure the bot is settled

        # Since this auto will hardly change if we're on separate sides of the field, we take the 
        # color argument and use it in this single condition instead of creating an entirely new
        # function with the same code. 
        if color == Color.RED:
            self.turn_for_auto(LEFT, 80, 10) # Turn 90 degrees to have the rear face a goal
        else:
            self.turn_for_auto(RIGHT, 80, 10) 
             
        wait(250, MSEC) # Let the bot rest

        self.auto_min() # Go backward until we get the mobile goal

        intake.spin_for(FORWARD, 5, SECONDS) # Score the rings while we're moving

        # We can also go a little further to make sure we're touching the middle ladder
        # This would consist of probably facing backwards so our mobile goal doesn't get
        # In the way, then running forward into a latter post. 

    def auto_direct_score(self):
        """
        Do nothing but go backward and score our preload. 

        Criteria:
            - Same as auto_min()
            - Requires a preload to be useful
        """
        self.auto_min()

        intake.spin_for(FORWARD, 3, SECONDS)

    def selector(self, competition: Competition):
        """
        Autonomous selector allows the controller user to select which auto they are
        going to use
        """
        # Global to make sure these variables can be used inside of the child functions
        global screen_should_be_refreshed, confirmed

        scr = brain.screen # A shortcut so we don't have to type it out each time
        screen_should_be_refreshed = True # To avoid unnecessary screen refreshes
        confirmed = False # If the auto is confirmed. 

        # Python allows functions to be defined inside of other functions, but why do this?
        # After all, we can simply define functions alongside of each other and call them through
        # the class. Nesting functions can also cause some janky behavior with variables.
        #
        # I wrote this code with nested functions, or functions defined inside of functions, for the 
        # sake of organization. These child functions (print_selected and screen_press) need to use 
        # variables from its parent function (selector), since its parent function starts changing variables
        # that are defined inside of its scope but still relative to what the child functions are doing.
        # The Auto class wasn't meant to manage the state of any single autonomous period, rather to organize
        # them all and give each method access to information they need. Since the state of the selector,
        # like variables such as `screen_should_be_refreshed` and `confirmed`, are only relevant to itself,
        # and the children functions are only relevant to the parent function, we group this all together in 
        # its own little ecosystem. It can be thought of as a poor man's class, just an easier way to organize
        # these functions rather than making an entire selector class. 
        
        def print_selected():
            """
            Print which auto is selected on the brain screen.

            The brain screen has a resolution of 480 x 272, but since the top 32 lines
            of the screen are used for the VEX status bar, there's a usable area of 
            480 x 240 to mess with. There is 480 usable pixels in the x range, and
            240 usable in the y range. The system starts from (0,0) being the bottom
            left of the screen. 
            """
            scr.clear_screen()
            scr.set_cursor(1, 0)
            scr.print("> " + self.available_autos[self.selected_auto][0])
            scr.new_line()
            scr.print("(GREEN) to confirm")
            scr.new_line()
            scr.print("(RED) to switch")

            # Draw_rectangle() has the parameters start_x, start_y, 
            # width, and height, where start_x and start_y are a point at the top-left
            # of the rectangle and width and height are how far down or right the rectangle
            # should expand. In this case, we make two equal squares each with a padding of
            # 10px from the sides of the screen, and a 10px gap between them. 
            scr.draw_rectangle(10, 120, 225, 110, color=Color.RED)
            scr.draw_rectangle(250, 120, 225, 110, color=Color.GREEN)

        def screen_press():
            """
            Handle when the screen is pressed, or our colored buttons are touched. 
            """
            touch_coords = (scr.x_position(), scr.y_position())

            if touch_coords[1] < 110: # If the screen is touched above where the buttons are defined, we don't care
                return
            
            # If x is less than 225, or the bottom-left of the screen is touched, we handle the red button actions
            if touch_coords[0] < 225: 
                self.selected_auto += 1

                if self.selected_auto >= len(self.available_autos):
                    self.selected_auto = 0
                
                global screen_should_be_refreshed
                screen_should_be_refreshed = True

            # If x is greater than 250, or the bottom-right of the screen is touched, we handle the green buttons
            elif touch_coords[0] > 250:
                scr.clear_screen()

                scr.set_cursor(3, 0)
                scr.print("Selected auto: " + self.available_autos[self.selected_auto][0])
                scr.new_line()
                scr.print("Ready to go! GHLF :)")

                global confirmed
                confirmed = True

            # Technically, we aren't handling entirely within the bounds of the buttons here. In these conditions,
            # We're checking if x < 225 and x > 250 and y < 110, which includes the padded area between the 
            # button and the screen. This isn't too important, and we can argue that if the padding was touched, the 
            # user wanted to perform the nearest action anyway. 
        
        brain.screen.pressed(screen_press) # Register the brain screen press event

        # While the autonomous is not confirmed, we refresh the screen if it needs refreshing. 
        while not confirmed and (not competition.is_competition_switch() or competition.is_driver_control()):
            # Only refresh the screen if the selected auto has changed, since it causes an annoying 
            # flashing effect if the screen is constantly refreshed. 
            if screen_should_be_refreshed:
                print_selected()
                global screen_should_be_refreshed
                screen_should_be_refreshed = False

            wait(250, MSEC)
        
        print("All done!")

    def run(self):
        """
        A dummy function that could be a lambda that's just used to run our selected auto 
        so we can define the Competition and have it ready to go before the auto is selected.
        """
        self.available_autos[self.selected_auto][1]()

        controller.screen.clear_screen()
        controller.screen.set_cursor(1, 1)
        controller.screen.print("Running Auto:")
        controller.screen.next_row()
        controller.screen.print(self.available_autos[self.selected_auto][0])

def controller_screen():
    """
    Write the temperatures of the motors to the screen of the controller. This is hardly
    useful during the competition, and could be more useful if managing a dynamic status
    screen that can simply say "OVERHEATING" if the temperature gets too high. 
    Having too much information available to the driver can be overwhelming, which
    is why a second or third teammate is necessary on the field. The driver focuses on
    driving the robot and listens to instructions from a teammate, who will gather information
    about the field in order to filter and decode it for use by the driver.

    A user from the VEX forum states motor current is cut in half at 55C, or about 131F. 
    We could use this temperature to warn the driver that the robot is overheating. 
    https://www.vexforum.com/t/v5-motor-temp-in-percent/52433/4
    """
    # Have the controller show the temperatures of the drivetrain motors, for some reason.
    scr = controller.screen
    overheating = False
    print("Hi")

    while True:
        # Gather all of the temperatures
        print("Loopig")

        for motor in motors:
            if motor.temperature() >= 55:
                overheating = True
                break
            else:
                overheating = False
        
        if overheating:
            scr.clear_screen()
            scr.set_cursor(1, 1)
            scr.print("!!! Warning !!!")
            scr.next_row()
            scr.print("Drivetrain Overheating")
        else:
            scr.clear_screen()
            scr.set_cursor(1, 1)
            scr.print("All good :)")


        wait(5, SECONDS) # Update the motor temperatures on the screen only every five seconds

def driver_control():
    """
    When it's driver control time, this function will run. This can be used to set up
    the robot and get it ready to be driven. In our case, everything is already set up
    so there's not much to do here.
    """
    optical.set_light(0) # Don't burn out the optical LED from autonomous
    Thread(controller_screen)

auto = Auto()

# Finally, define and run the Competition class which communicates with the field controller. It'll run the 
# driver_control() method when it's time for driver control, and the auto.run() method when it's time for auto
competition = Competition(driver_control, auto.run)

auto.selector(competition)


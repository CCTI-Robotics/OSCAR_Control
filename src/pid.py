from vex import *

mgR_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
mgR_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
mgR = MotorGroup(mgR_motor_a, mgR_motor_b)
mgL_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False)
mgL_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
mgL = MotorGroup(mgL_motor_a, mgL_motor_b)
drivetrain = DriveTrain(mgL, mgR, 319.19, 330, 320, MM, 1)
gps = Gps(Ports.PORT17, 6.0, 7.0, INCHES, 180)

class Vector:
    def __init__(self, x: int, y: int):
        self.x: int = x
        self.y: int = y

    def magnitude(self):
        # Calculate the magnitude of the vector. 
        # Magnitude is the distance of the object from the origin.
        # Magnitude is sqrt(X^2 + Y^2)
        return ((self.x ** 2) + (self.y ** 2)) ** 0.5
    
    @staticmethod
    def distance(vec1: "Vector", vec2: "Vector"): 
        # Calculate the distance between one vector and another. 
        # Distance formula: sqrt((x2 - x1)^2 + (y2 - y1)^2)
        return (((vec2.x - vec1.x) ** 2) + ((vec2.y - vec1.y)) ** 2) ** 0.5
    
    def __str__(self) -> str:
        print("({}, {})".format(self.x, self.y))

def gps_pos():
    return Vector(gps.x_position(), gps.y_position())


class PIDwithGPS:
    kP = 0.1
    kI = 0
    kD = 0

    def drive(self, end_vec: Vector, turn: bool = False):
        integral = 0
        error = 0
        prev_error = 0
        derivative = 0
        power = 0

        lowest_dist = Vector.distance(gps_pos(), end_vec) + 100 # Continue this

        while True:
            error = Vector.distance(gps_pos(), end_vec)
            integral += error
            print(error)

            if error == 0 or error < 20:
                integral = 0
            
            if error > 1000:
                integral = 0

            derivative = error - prev_error
            prev_error = error

            power = (error * self.kP) + (integral * self.kI) + (derivative * self.kD)
            
            if not turn:
                drivetrain.drive(FORWARD, power, PERCENT)
            else:
                drivetrain.turn(RIGHT, power, PERCENT)

            wait(15, MSEC)

pid = PIDwithGPS()

pos = gps_pos()
pid.drive(Vector(-1488.433, -288.3048))
print(pos)

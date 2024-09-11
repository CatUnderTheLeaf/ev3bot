#!/usr/bin/env python3

from ev3dev2.motor import MoveDifferential, OUTPUT_A, OUTPUT_D, SpeedRPS, SpeedRPM, SpeedPercent, LargeMotor, SpeedNativeUnits
from ev3dev2.wheel import EV3Tire
from ev3dev2.sound import Sound
from ev3dev2.led import Leds

class MoveSteerDiff(MoveDifferential):
    """
    A combination of MoveDifferential and MoveSteering clasess
    to control the robot with `cmd_vel` message type

    Args:
        MoveDifferential (_type_): _description_
    """
    def __init__(self, left_motor, right_motor, wheel_distance_mm):
        MoveDifferential.__init__(self, left_motor, right_motor, EV3Tire, wheel_distance_mm)

    def steer_on(self, steering, speed):
        """
        Start rotating the motors according to the provided ``steering`` and
        ``speed`` forever.
        """
        (left_speed, right_speed) = self.get_speed_steering(steering, speed)
        MoveDifferential.on(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed))

    def get_speed_steering(self, steering, speed):
        """
        Calculate the speed_sp for each motor in a pair to achieve the specified
        steering. Note that calling this function alone will not make the
        motors move, it only calculates the speed. A run_* function must be called
        afterwards to make the motors move.

        steering [-100, 100]:
            * -100 means turn left on the spot (right motor at 100% forward, left motor at 100% backward),
            *  0   means drive in a straight line, and
            *  100 means turn right on the spot (left motor at 100% forward, right motor at 100% backward).

        speed:
            The speed that should be applied to the outmost motor (the one
            rotating faster). The speed of the other motor will be computed
            automatically.
        """

        assert steering >= -100 and steering <= 100,\
            "{} is an invalid steering, must be between -100 and 100 (inclusive)".format(steering)

        # We don't have a good way to make this generic for the pair... so we
        # assume that the left motor's speed stats are the same as the right
        # motor's.
        speed = self.left_motor._speed_native_units(speed)
        left_speed = speed
        right_speed = speed
        speed_factor = (50 - abs(float(steering))) / 50

        if steering >= 0:
            right_speed *= speed_factor
        else:
            left_speed *= speed_factor

        return (left_speed, right_speed)
    
    def steer_on_for_rotations(self, steering, speed, rotations, brake=True, block=True):
        """
        Rotate the motors according to the provided ``steering``.

        The distance each motor will travel follows the rules of :meth:`MoveTank.on_for_rotations`.
        """
        (left_speed, right_speed) = self.get_speed_steering(steering, speed)
        MoveDifferential.on_for_rotations(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), rotations, brake,
                                  block)

    def steer_on_for_degrees(self, steering, speed, degrees, brake=True, block=True):
        """
        Rotate the motors according to the provided ``steering``.

        The distance each motor will travel follows the rules of :meth:`MoveTank.on_for_degrees`.
        """
        (left_speed, right_speed) = self.get_speed_steering(steering, speed)
        MoveDifferential.on_for_degrees(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), degrees, brake,
                                block)

    def steer_on_for_seconds(self, steering, speed, seconds, brake=True, block=True):
        """
        Rotate the motors according to the provided ``steering`` for ``seconds``.
        """
        (left_speed, right_speed) = self.get_speed_steering(steering, speed)
        MoveDifferential.on_for_seconds(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), seconds, brake,
                                block)


class LegoBot(MoveSteerDiff):
    
    def __init__(self, left_motor, right_motor, wheel_distance_mm):
        MoveSteerDiff.__init__(self, left_motor, right_motor, wheel_distance_mm)
        """ 
        LegoBot Class inherits all usefull stuff for differential drive
        and adds sound, LEDs
         """
        self.leds = Leds()
        self.sound = Sound()
        self.leds.set_color("LEFT", "BLACK")
        self.leds.set_color("RIGHT", "BLACK")

        # Startup sequence
        self.sound.play_song((('C4', 'e'), ('D4', 'e'), ('E5', 'q')))
        self.leds.set_color("LEFT", "GREEN")
        self.leds.set_color("RIGHT", "GREEN")

        print('Hello, my name is EV3!')

        self.sound.speak('Hello, my name is EV3!')

    def turn_off(self):
        """
        A sequence to shutdown the robot

        """      
        # stop odometry thread
        # self.odometry_stop()

        # Shutdown sequence
        self.sound.play_song((('E5', 'e'), ('C4', 'e')))
        self.leds.set_color("LEFT", "BLACK")
        self.leds.set_color("RIGHT", "BLACK")
    
    def move(self,steering, speed):
        """
        Rotates both motors with the specified
        steering and speed

        Args:
            steering (int): angular velocity
            speed (int): linear velocity
        """
        self.steer_on(steering, SpeedPercent(speed))      
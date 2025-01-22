#!/usr/bin/env python3

from ev3dev2.motor import MoveDifferential, OUTPUT_A, OUTPUT_D, SpeedRPS, SpeedRPM, SpeedPercent, LargeMotor, SpeedNativeUnits
from ev3dev2.wheel import EV3Tire
from ev3dev2.sound import Sound
from ev3dev2.led import Leds

import math

class MoveSteerDiff(MoveDifferential):
    """
    A combination of MoveDifferential and MoveSteering clasess
    to control the robot with `cmd_vel` message type

    Args:
        MoveDifferential (_type_): _description_
    """
    def __init__(self, left_motor, right_motor, wheel_distance_mm):
        MoveDifferential.__init__(self, left_motor, right_motor, EV3Tire, wheel_distance_mm)
        # MoveDifferential.odometry_start()

    def steer_on(self, angular, speed):
        """
        Start rotating the motors according to the provided ``angular velocity`` and
        ``speed`` forever.
        """
        (left_speed, right_speed) = self.get_speed_steering(angular, speed)
        MoveDifferential.on(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed))

    def get_speed_steering(self, angular, linear):
        """
        Calculate the speed_sp for each motor in a pair to achieve the specified
        angular velocity. Note that calling this function alone will not make the
        motors move, it only calculates the speed. A run_* function must be called
        afterwards to make the motors move.

        angular velocity [-pi, pi] in rad/s:
            * -pi means turn left on the spot (right motor at 100% forward, left motor at 100% backward),
            *  0   means drive in a straight line, and
            *  pi means turn right on the spot (left motor at 100% forward, right motor at 100% backward).
                        
        linear velocity in m/s:
            V = 2rN or V = DN, 
            where
                V - velocity of the body rotating m/s
                r - radius of rotation
                D - diameter of rotation
                N - no. of rotation/revolution per minute
        """

        assert angular > -math.pi or angular < math.pi, \
            "Input {} must be in the range [-pi, pi]".format(angular)
        
        # transform steering to [-100, 100]
        steering = -100 + ((angular + math.pi) * 100 / math.pi)

        # We don't have a good way to make this generic for the pair... so we
        # assume that the left motor's speed stats are the same as the right
        # motor's.

        speedRPM = SpeedRPM(linear * 1000 / self.wheel.diameter_mm)
        speed = self.left_motor._speed_native_units(speedRPM)
        left_speed = speed
        right_speed = speed
        speed_factor = (50 - abs(float(steering))) / 50

        if steering >= 0:
            right_speed *= speed_factor
        else:
            left_speed *= speed_factor
        print("transformed left_speed: {}, right_speed: {}".format(left_speed, right_speed))
        # linear=0.7, angular=1, left_speed=97, right_speed=35
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
    
    def move(self, angular, linear):
        """
        Rotates both motors with the specified
        steering and speed

        Args:
            angular (float): angular velocity [-pi, pi] in rad/s
            linear (float): linear velocity, m/s
        """
        print("angular: {}, linear: {}".format(angular, linear))
        self.steer_on(angular, linear)
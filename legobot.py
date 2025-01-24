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
        
        # limits for the ev3 large motors in mm/s
        self.wheel_max_velocity = 170*math.pi*self.wheel.diameter_mm/60 # max rpm=170, v_man = 384.5 mm/s
        self.wheel_min_velocity = 30*math.pi*self.wheel.diameter_mm/60  # min rpm=30, v_min = 67.8 mm/s

        self.target_linear_velocity = 0 # mm/s
        self.target_angular_velocity = 0 # rad/s
        """ 
        LegoBot Class inherits all usefull stuff for differential drive
        and adds sound, LEDs
         """
        self.leds = Leds()
        self.sound = Sound()
        self.leds.set_color("LEFT", "BLACK")
        self.leds.set_color("RIGHT", "BLACK")

        # Startup sequence
        # self.sound.play_song((('C4', 'e'), ('D4', 'e'), ('E5', 'q')))
        self.leds.set_color("LEFT", "GREEN")
        self.leds.set_color("RIGHT", "GREEN")

        print('Hello, my name is EV3!')

        # self.sound.speak('Hello, my name is EV3!')

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
    
    def move(self, linear, angular, dt):
        """
        Rotates both motors with the specified
        steering and speed

        Args:
            angular (float): angular velocity in rad/s
            linear (float): linear velocity, m/s
            dt (): time interval to run the motors
        """
        # every linear velocity in ev3dev is in mm/s
        self.target_linear_velocity = linear*1000
        self.target_angular_velocity = angular
        # print("linear: {}, angular: {}".format(self.target_linear_velocity, self.target_angular_velocity))
        vl, vr = self.get_wheel_speeds()
        # print("transformed to vl: {}, vr: {}".format(vl, vr))
        
        # actual robot move
        # self.on_for_seconds(vl, vr, dt, False, False)
        self.on(vl, vr)
    
    def diff2uni(self, vl, vr):
        """
        Calculate the linear and angular velocity of the robot
        based on the speed of the left and right wheels
        
        Args:
            vl (int): speed of the left wheel in mm/s
            vr (int): speed of the right wheel in mm/s
        """

        v = (vl+vr) / 2
        w = (vr-vl) / self.wheel_distance_mm
        return v, w

    def uni2diff(self, v, w):
        """
        Calculate the speed of the left and right wheels
        based on the linear and angular velocity

        Args:
            v (float): linear velocity, mm/s
            w (float): angular velocity in rad/s
        """
        vl = v - w * self.wheel_distance_mm / 2
        vr = v + w * self.wheel_distance_mm / 2
        return vl, vr

    def get_wheel_speeds(self):
        """ 
            The robot's motors have a maximum angular velocity, and the motors stall at low speeds. 
            Suppose that we pick a linear velocity v that requires the motors to spin at 90% power. 
            Then, we want to change ω from 0 to some value that requires 20% more power from the right motor, 
            and 20% less power from the left motor. This is not an issue for the left motor, 
            but the right motor cannot turn at a capacity greater than 100%. 
            The results is that the robot cannot turn with the ω specified by our controller.

            Since PID controllers focus more on steering than on controlling the linear velocity, 
            we want to prioritize ω over v in situations, where we cannot satisfy ω with the motors. 
            In fact, we will simply reduce v until we have sufficient headroom to achieve ω with the robot. 
            The function is designed to ensure that ω is achieved even if the original combination of v and ω exceeds the maximum vl and vr.
        """     
        # This code is taken directly from Sim.I.Am project
        # and remade to adjust to the new variables
        
        if self.target_linear_velocity == 0:
            
            # Robot is stationary, so we can either not rotate, or
            # rotate with some minimum/maximum angular velocity

            w_min = (2*self.wheel_min_velocity)/self.wheel_distance_mm
            w_max = (2*self.wheel_max_velocity)/self.wheel_distance_mm
            
            if abs(self.target_angular_velocity) > w_min:
                w = math.copysign(max(min(abs(self.target_angular_velocity), w_max), w_min), self.target_angular_velocity)
            else:
                w = 0
            
            limited_vl, limited_vr = self.uni2diff(0,w)
            
        else:
            # 1. Limit v,w to be possible in the range [vel_min, vel_max]
            # (avoid stalling or exceeding motor limits)
            v_lim = max(min(abs(self.target_linear_velocity), self.wheel_max_velocity), self.wheel_min_velocity)
            w_lim = max(min(abs(self.target_angular_velocity), ((self.wheel_max_velocity - self.wheel_min_velocity)/self.wheel_distance_mm)), 0)
            
            # 2. Compute the desired curvature of the robot's motion
            # print("limited v: {}, w: {}".format(v_lim, w_lim))
            vl,vr = self.uni2diff(v_lim, w_lim)
            # print("transform to vl: {}, vr: {}".format(vl, vr))
            
            # 3. Find the max and min vel_r/vel_l
            v_lr_max = max(vl, vr)
            v_lr_min = min(vl, vr)
            
            # 4. Shift vr and vl if they exceed max/min vel
            if (v_lr_max > self.wheel_max_velocity):
                vr -= v_lr_max - self.wheel_max_velocity
                vl -= v_lr_max - self.wheel_max_velocity
            elif (v_lr_min < self.wheel_min_velocity):
                vr += self.wheel_min_velocity - v_lr_min
                vl += self.wheel_min_velocity - v_lr_min
            # print("after shift vl: {}, vr: {}".format(vl, vr))
            # 5. Fix signs (Always either both positive or negative)
            v_shift, w_shift = self.diff2uni(vl,vr)
            
            v = math.copysign(v_shift,self.target_linear_velocity)
            w = math.copysign(w_shift,self.target_angular_velocity)
            # print("with sign v: {}, w: {}".format(v, w))
            limited_vl, limited_vr = self.uni2diff(v,w)
            
            
        # limited_vl,limited_vr = self.uni2diff(self.target_linear_velocity, self.target_angular_velocity)
        # print("with sign limited_vl: {}, limited_vr: {}".format(limited_vl, limited_vr))
        vl_rpm = (limited_vl * 60) / (math.pi * self.wheel.diameter_mm)
        vr_rpm = (limited_vr * 60) / (math.pi * self.wheel.diameter_mm)

        # return velocities as SpeedValue objects
        return (SpeedRPM(vl_rpm), SpeedRPM(vr_rpm))
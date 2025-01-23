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
        # limits for the robot large motors
        self.max_speed_rpm = 170
        self.min_speed_rpm = 30
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
    
    def move(self, linear, angular, dt):
        """
        Rotates both motors with the specified
        steering and speed

        Args:
            angular (float): angular velocity in rad/s
            linear (float): linear velocity, m/s
            dt (): time interval to run the motors
        """
        print("linear: {}, angular: {}".format(linear, angular))
        # self.steer_on(angular, linear)
        # linear=0.7, angular=1, left_speed=97, right_speed=35

        # multiply linear velocity by 1000 to convert it to mm/s
        vl, vr = self.get_wheel_speeds(linear*1000, angular)
        
        # actual robot move
        self.on_for_seconds(vl, vr, dt, False, False)
    
    def get_wheel_speeds(self, linear, angular):
        """
        Calculate the speed of the left and right wheels
        based on the linear and angular velocity

        Args:
            angular (float): angular velocity in rad/s
            linear (float): linear velocity, mm/s
        """
        # calculate the linear velocity of the left and right wheels
        vl = linear - angular * self.wheel_distance_mm / 2
        vr = linear + angular * self.wheel_distance_mm / 2

        vl_rpm = (vl * 60) / (math.pi * self.wheel.diameter_mm)
        vr_rpm = (vr * 60) / (math.pi * self.wheel.diameter_mm)

        vl_limited  = self.ensure_wheel_rpm(vl_rpm)
        vr_limited = self.ensure_wheel_rpm(vr_rpm)

        return (SpeedRPM(vl_limited), SpeedRPM(vr_limited))
    
    def ensure_wheel_rpm(self, speed):
        """
        Ensure the wheel speed is within the limits

        Args:
            speed (int): the speed to be checked
        """
        # TODO rewrite it
        return max(-self.max_speed_rpm, min(self.max_speed_rpm, speed))
    
# from sim i am
    # def ensure_w(self,v_lr):        
    #     """ 
    #         The robot’s motors have a maximum angular velocity, and the motors stall at low speeds. 
    #         Suppose that we pick a linear velocity v that requires the motors to spin at 90% power. 
    #         Then, we want to change ω from 0 to some value that requires 20% more power from the right motor, 
    #         and 20% less power from the left motor. This is not an issue for the left motor, 
    #         but the right motor cannot turn at a capacity greater than 100%. 
    #         The results is that the robot cannot turn with the ω specified by our controller.

    #         Since PID controllers focus more on steering than on controlling the linear velocity, 
    #         we want to prioritize ω over v in situations, where we cannot satisfy ω with the motors. 
    #         In fact, we will simply reduce v until we have sufficient headroom to achieve ω with the robot. 
    #         The function is designed to ensure that ω is achieved even if the original combination of v and ω exceeds the maximum vl and vr.
    #     """     
    #     # This code is taken directly from Sim.I.Am project
    #     v_max = self.robot.wheels.max_velocity
    #     v_min = self.robot.wheels.min_velocity
       
    #     R = self.robot.wheels.radius 
    #     L = self.robot.wheels.base_length 
        
    #     def diff2uni(vl,vr):
    #         return (vl+vr) * R/2, (vr-vl) * R/L
        
    #     v, w = diff2uni(*v_lr)
        
    #     if v == 0:
            
    #         # Robot is stationary, so we can either not rotate, or
    #         # rotate with some minimum/maximum angular velocity

    #         w_min = R/L*(2*v_min)
    #         w_max = R/L*(2*v_max)
            
    #         if abs(w) > w_min:
    #             w = copysign(max(min(abs(w), w_max), w_min), w)
    #         else:
    #             w = 0
            
    #         return self.uni2diff((0,w))
            
    #     else:
    #         # 1. Limit v,w to be possible in the range [vel_min, vel_max]
    #         # (avoid stalling or exceeding motor limits)
    #         v_lim = max(min(abs(v), (R/2)*(2*v_max)), (R/2)*(2*v_min))
    #         w_lim = max(min(abs(w), (R/L)*(v_max - v_min)), 0)
            
    #         # 2. Compute the desired curvature of the robot's motion
            
    #         vl,vr = self.uni2diff((v_lim, w_lim))
            
    #         # 3. Find the max and min vel_r/vel_l
    #         v_lr_max = max(vl, vr)
    #         v_lr_min = min(vl, vr)
            
    #         # 4. Shift vr and vl if they exceed max/min vel
    #         if (v_lr_max > v_max):
    #             vr -= v_lr_max - v_max
    #             vl -= v_lr_max - v_max
    #         elif (v_lr_min < v_min):
    #             vr += v_min - v_lr_min
    #             vl += v_min - v_lr_min
            
    #         # 5. Fix signs (Always either both positive or negative)
    #         v_shift, w_shift = diff2uni(vl,vr)
            
    #         v = copysign(v_shift,v)
    #         w = copysign(w_shift,w)
            
    #         return self.uni2diff((v,w))

    # wheel 43.2 mm diameter, 21 mm width
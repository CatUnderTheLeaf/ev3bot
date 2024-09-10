#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, MoveDifferential, SpeedRPS, SpeedRPM, SpeedPercent, LargeMotor
from ev3dev2.wheel import EV3Tire
from ev3dev2.sound import Sound
from ev3dev2.led import Leds

class LegoBot(MoveDifferential):
    
    def __init__(self, wheel_distance_mm):
        MoveDifferential.__init__(self, OUTPUT_A, OUTPUT_D, EV3Tire, wheel_distance_mm)
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
        # stop odometry thread
        # self.odometry_stop()

        # Shutdown sequence
        self.sound.play_song((('E5', 'e'), ('C4', 'e')))
        self.leds.set_color("LEFT", "BLACK")
        self.leds.set_color("RIGHT", "BLACK")
    
    def move(self,steering, speed, seconds):
        self.on_for_seconds(steering, SpeedPercent(speed), seconds)
        
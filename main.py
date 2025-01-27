#!/usr/bin/env python3

import os
from os import strerror

import sys

import logging
import yaml
import time

from legobot import LegoBot
from agt import AlexaGadget

import paho.mqtt.client as mqtt


# set logger to display on both EV3 Brick and console
logging.basicConfig(level=logging.INFO, stream=sys.stdout, format='%(message)s')
logging.getLogger().addHandler(logging.StreamHandler(sys.stderr))
logger = logging.getLogger(__name__)

# class AlexaBot(AlexaGadget):
#     def __init__(self):
#         super().__init__()
    
#     def on_connected(self, device_addr):
#         """
#         Gadget connected to the paired Echo device.
#         :param friendly_name: the friendly name of the gadget that has connected to the Echo device
#         """
#         self.leds.set_color("LEFT", "GREEN")
#         self.leds.set_color("RIGHT", "GREEN")
#         print("connected to Echo device")

#     def on_disconnected(self, device_addr):
#         """
#         Gadget disconnected from the paired Echo device.
#         :param friendly_name: the friendly name of the gadget that has disconnected from the Echo device
#         """
#         self.leds.set_color("LEFT", "BLACK")
#         self.leds.set_color("RIGHT", "BLACK")
#         print("disconnected from Echo device")

#     def on_alexa_gadget_statelistener_stateupdate(self, directive):
#         """
#         Listens for the wakeword state change and react by turning on the LED.
#         :param directive: contains a payload with the updated state information from Alexa
#         """
#         color_list = ['BLACK', 'AMBER', 'YELLOW', 'GREEN']
#         for state in directive.payload.states:
#             if state.name == 'wakeword':

#                 if state.value == 'active':
#                     print("Wake word active", file=sys.stderr)
#                     self.sound.play_song((('A3', 'e'), ('C5', 'e')))
#                     for i in range(0, 4, 1):
#                         self.leds.set_color("LEFT", color_list[i], (i * 0.25))
#                         self.leds.set_color("RIGHT", color_list[i], (i * 0.25))
#                         # time.sleep(0.25)
#                     # self.bot.move(0.25)
#                 elif state.value == 'cleared':
#                     print("Wake word cleared", file=sys.stderr)
#                     self.sound.play_song((('C5', 'e'), ('A3', 'e')))
#                     for i in range(3, -1, -1):
#                         self.leds.set_color("LEFT", color_list[i], (i * 0.25))
#                         self.leds.set_color("RIGHT", color_list[i], (i * 0.25))
                        # time.sleep(0.25)

if __name__ == '__main__':
     
    # set large letters on ev3 display
    os.system('setfont Lat15-TerminusBold14')
    client = mqtt.Client()
        
    # load configuration
    try:
        with open("config.yaml", mode="r") as f:
            config = yaml.safe_load(f)

        if config is None:
            print('empty config file')
            sys.exit()

        # create a robot instance
        bot = LegoBot(left_motor=config['left_motor'],
                    right_motor=config['right_motor'],
                    wheel_distance_mm=config['wheel_separation'])
        
        # MQTT subscriber functions
        def on_connect(client, userdata, flags, rc):
            """
            connect to the MQTT client and subscribe to a topic

            Args:
                client (Client) - the client instance for this callback
                userdata - the private user data
                connect_flags (ConnectFlags) - the flags for this connection
                reason_code (ReasonCode) - the connection reason code received from the broken
            """
            print("Connected with result code "+str(rc))
            client.subscribe(config['legobot_cmd'])

        def on_message(client, userdata, msg):
            """
            recieve `legobot_cmd` message and 
            move the robot accordingly

            Args:
                client (Client) - the client instance for this callback
                userdata - the private user data
                msg (MQTTMessage) - the received message
            """
            
            message = msg.payload.decode()
            command = yaml.safe_load(message)
            print(command)
            print("-------")
            print("time differences")
            print(time.time() - float(command['sec']))
            # print(time.time_ns() - int(command['nanosec']))
            print("-------")
            # there is no `match...case` in Python 3.5 :(
            if command['cmd'] == 'drive':
                print('driving on command')
                bot.move(float(command['linear']), float(command['angular']))        
            elif command['cmd'] == 'stop':
                bot.stop()
            elif command['cmd'] == 'speak':
                bot.sound.speak(command['text'])
            elif command['cmd'] == 'quit':
                client.disconnect()
                bot.turn_off()
            else:
                print('command not found')

        # create and run MQTT client to receive messages
        client.connect(config['broker_ip'], config['port'], config['keep_alive'])

        client.on_connect = on_connect
        client.on_message = on_message

        client.loop_start()

        # time interval for publishing odometry
        dt = 1 / config['rate']
        while True:
            #code for publishing
            client.publish(config['legobot_odom'], bot.get_odometry())
            time.sleep(dt)

    except OSError as error:
        print(strerror(error.errno))
    except yaml.YAMLError as exc:
        print(exc)

    finally:
        client.loop_stop()

    

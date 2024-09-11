#!/usr/bin/env python3

import os
from os import strerror

import sys

import logging
import yaml

from legobot import LegoBot

import paho.mqtt.client as mqtt


# set logger to display on both EV3 Brick and console
logging.basicConfig(level=logging.INFO, stream=sys.stdout, format='%(message)s')
logging.getLogger().addHandler(logging.StreamHandler(sys.stderr))
logger = logging.getLogger(__name__)


if __name__ == '__main__':
     
    # set large letters on ev3 display
    os.system('setfont Lat15-TerminusBold14')
    
    # load configuration
    try:
        with open("config.yaml", mode="r") as f:
            config = yaml.safe_load(f)
    except OSError as error:
        print(strerror(error.errno))
    except yaml.YAMLError as exc:
        print(exc)

    if config is None:
        print('empty config file')
        sys.exit()

    DIST_BTW_WHEELS = config['stud_num'] * config['stud_mm']
   
    # create a robot instance
    bot = LegoBot(config['left_motor'], config['right_motor'], DIST_BTW_WHEELS)


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
        client.subscribe(config['cmd_vel'])

    def on_message(client, userdata, msg):
        """
        recieve `cmd_vel` message and 
        move the robot accordingly

        Args:
            client (Client) - the client instance for this callback
            userdata - the private user data
            message (MQTTMessage) - the received message
        """
        if msg.payload.decode() == 'q':
            client.disconnect()
            print('turning off')
            bot.turn_off()
        else:
            speed, steer = msg.payload.decode().split(' ')
            if steer == '0' and speed == '0':
                print('stopping')
                bot.stop()
            else:
                print(msg.payload.decode())
                bot.move(int(steer), int(speed))

    # create and run MQTT client to receive messages
    client = mqtt.Client()
    client.connect(config['broker_ip'], config['port'], config['keep_alive'])

    client.on_connect = on_connect
    client.on_message = on_message

    client.loop_forever()

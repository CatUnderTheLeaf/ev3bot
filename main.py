#!/usr/bin/env python3

import os

import sys
import logging

from legobot import LegoBot


# set logger to display on both EV3 Brick and console
logging.basicConfig(level=logging.INFO, stream=sys.stdout, format='%(message)s')
logging.getLogger().addHandler(logging.StreamHandler(sys.stderr))
logger = logging.getLogger(__name__)


if __name__ == '__main__':
     
    # set large letters on ev3 display
    os.system('setfont Lat15-TerminusBold14')
    
    STUD_MM = 8
    DIST_BTW_WHEELS = 15 * STUD_MM
   
    bot = LegoBot(DIST_BTW_WHEELS)

    # move 30# right, 75% speed, 10 seconds
    bot.move(30, 75, 10)

    bot.turn_off()
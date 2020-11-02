#!/usr/bin/env python3

from flir_ptu.ptu import PTU
from vision_utils.logger import get_logger
import time
logger =  get_logger()

x = PTU("192.168.1.110", 4000, debug=False)
x.connect()

# reset PTU
x.reset()
x.wait()

# having fun with sines
import numpy as np
sin = np.arcsin(np.arange(-1,1.0,0.01)) * 180/np.pi
sin_inv = np.arcsin(np.arange(1.0,-1,0.01)) * 180/np.pi

# set upper speed limit
x.pan_speed_max(11000)
x.wait()

x.set_position_mode()
x.wait()

x.pan_speed(1000)
x.wait()

x.tilt_speed(1000)
x.wait()

# demo of different speeds
for i in range(20):

    x.pan_angle(-25)
    time.sleep(2)

    x.tilt_angle(5)
    time.sleep(2)

    x.pan_angle(0)
    time.sleep(2)

    x.tilt_angle(-10)
    time.sleep(2)

    x.pan_angle(25)
    time.sleep(2)

    x.tilt_angle(-20)
    time.sleep(2)

x.stream.close()

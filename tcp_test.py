#!/usr/bin/env python3

from flir_ptu.ptu import PTU
from vision_utils.logger import get_logger
import time
logger =  get_logger()

x = PTU("192.168.0.110", 4000, debug=False)
x.connect()

# reset PTU
x.reset()
x.wait()

# having fun with sines
import numpy as np
sin = np.arcsin(np.arange(-1,1.0,0.01)) * 180/np.pi
sin_inv = np.arcsin(np.arange(1.0,-1,0.01)) * 180/np.pi

# set upper speed limit
x.pan_speed_max(16000)
x.wait()

# demo of different speeds
for speed in [2000, 4000, 8000, 16000]:
    x.pan_speed()
    x.pan_speed(speed)
    x.wait()

    x.pan_angle(-90)
    x.wait()

    x.pan_offset(-100)
    x.wait()

    x.pan_offset(100)
    x.wait()

    x.pan_angle(90)
    x.wait()

    time.sleep(1)

x.stream.close()

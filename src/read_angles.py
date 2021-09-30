#!/usr/bin/env python3

from flir_ptu.ptu import PTU
#from vision_utils.logger import get_logger
#from vision_utils.timing import get_timestamp
import time
#logger =  get_logger()

x = PTU("192.168.0.110", 4000, debug=False)
x.connect()

# reset PTU
# x.reset()
# x.wait()

# set upper speed limit
while(True):
    print(f"{get_timestamp()} pan: {x.pan()}")
    #logger.debug(f"{get_timestamp()} pan: {x.pan()}")
    x.wait()
    print(f"{get_timestamp()} tilt: {x.tilt()}")
    #logger.debug(f"{get_timestamp()} tilt: {x.tilt()}")
    x.wait()
    time.sleep(1)

x.stream.close()

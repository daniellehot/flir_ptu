#!/usr/bin/env python3

from flir_ptu.ptu import PTU
import time
import rospy
from simple_pid import PID
from sensor_msgs.msg import JointState
from vision_utils.logger import get_logger
import numpy as np

logger =  get_logger()



global position
position = [0.0, 0.0]

def state_cb(msg):
    if msg.name == ["ptu_panner", "ptu_tilter"]:
        global position
        position = msg.position
    
        

rospy.init_node("PTU_node")
rospy.Subscriber("/joint_states", JointState, state_cb)

pid = PID(0.2, 0.5, 1, setpoint=0)

x = PTU("192.168.1.110", 4000, debug=False)
x.connect()

# x.reset()
# x.wait()

# set upper speed limit
print(x.pan_speed_max())
x.pan_speed_max(8000)
x.wait()

# read accel
x.pan_accel()
x.wait()

# print(x.pan_angle())
# x.pan_angle(10)
# x.wait()

v = position[0]*180/np.pi


rate = rospy.Rate(50) # 10hz
while not rospy.is_shutdown():
    v = x.pan_angle()
    control = pid(v)
    x.pan_angle(control)
    
    v = position[0]*180/np.pi
    print("v: ", v)
    print("control: ", control)
    rate.sleep()
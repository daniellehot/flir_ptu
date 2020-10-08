#!/usr/bin/env python3
import numpy as np
from flir_ptu.ptu import PTU
from vision_utils.logger import get_logger
import time
logger =  get_logger()

x = PTU("192.168.1.110", 4000, debug=False)
x.connect()

x.reset()
x.wait()

# set upper speed limit
x.pan_speed_max(16000)
x.wait()

# read accel
x.pan_accel()
x.wait()

x.tilt_speed()
x.tilt_speed(2000)
x.wait()

x.tilt_angle(-25)
x.wait()

position = [0.0, 0.0]

angle = 0

def state_cb(msg):
    global position
    if msg.name == ["ptu_panner", "ptu_tilter"]:
        position = msg.position

def angle_cb(msg):
    global angle
    angle = msg.data
    print("angle data: ",msg.data)

import rospy
from simple_pid import PID
from sensor_msgs.msg import JointState
from vision_utils.logger import get_logger
import numpy as np
from std_msgs.msg import Float32

rospy.init_node("PTU_node")
rospy.Subscriber("/joint_states", JointState, state_cb)
rospy.Subscriber("/person_angle", Float32, angle_cb)

pid = PID(0.2, 0.5, 1, setpoint=0)

v = position[0]*180/np.pi
pid.output_limits = (-168.0,168.0)

rate = rospy.Rate(50) # 50hz
while not rospy.is_shutdown():
    pid.setpoint = angle
    control = pid(v)

    print(round(control,0))
    x.pan_angle(round(control,0))
    v = int(round(position[0]*180/np.pi,0))

    rate.sleep()

x.stream.close()

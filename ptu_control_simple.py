#!/usr/bin/env python3
import numpy as np
from flir_ptu.ptu import PTU
from vision_utils.logger import get_logger
import time
logger =  get_logger()

x = PTU("192.168.1.110", 4000, debug=False)
x.connect()

# x.reset()
# x.wait()

# set upper speed limit
x.pan_speed_max(16000)
x.wait()

x.set_position_mode()
x.wait()


position = [0.0, 0.0]

angle = 0
tilt_angle = 0

def state_cb(msg):
    global position
    if msg.name == ["ptu_panner", "ptu_tilter"]:
        position = msg.position

def angle_cb(msg):
    global angle
    angle = msg.data
    print("pan angle: ", angle)

def tilt_angle_cb(msg):
    global tilt_angle
    tilt_angle = msg.data
    print("tilt angle: ", tilt_angle)



import rospy
# from simple_pid import PID
from sensor_msgs.msg import JointState
from vision_utils.logger import get_logger
import numpy as np
from std_msgs.msg import Float32
import math

rospy.init_node("PTU_node")
rospy.Subscriber("/joint_states", JointState, state_cb)
rospy.Subscriber("/ref_pan_angle", Float32, angle_cb)
rospy.Subscriber("/ref_tilt_angle", Float32, tilt_angle_cb)
pub = rospy.Publisher("/control_output_pan", Float32, queue_size=1)
pub_tilt = rospy.Publisher("/control_output_tilt", Float32, queue_size=1)



v_pan = position[0]*180/np.pi
v_tilt = position[1]*180/np.pi



x.set_speed_mode()
x.wait()

X = np.linspace(-np.pi*4, np.pi*4, num=4000)

# numpy sine values
y = np.sin(X) * 180/np.pi
counter = 0
rate = rospy.Rate(50)

while not rospy.is_shutdown():
    # if counter < len(y):
    #     ref = y[counter]
    #     counter += 1
    ref = 45
    tilt_ref = -30


    error = ref - v_pan
    error_tilt = tilt_ref - v_tilt

    pub.publish(ref)

    if abs(error) >= 3:
        x.pan_angle(ref)

    pub.publish(ref)


    if abs(error_tilt) >= 3:
        x.tilt_angle(tilt_ref)
        
    pub_tilt.publish(tilt_ref)



    # x.pan_angle(control_pan)
    v_pan = position[0]*180/np.pi

    # x.tilt_angle(control_tilt)
    v_tilt = position[1]*180/np.pi
    rate.sleep()



x.stream.close()

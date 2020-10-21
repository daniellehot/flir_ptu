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

# read accel
x.pan_accel()
x.wait()

# x.tilt_speed()
# x.tilt_speed(2000)
# x.wait()

# x.pan_speed()
# x.pan_speed(2000)
# x.wait()

# x.set_speed_mode()
# x.wait()

x.set_position_mode()
x.wait()

# x.pan_angle(45)
# x.wait()

# x.tilt_angle(-25)
# x.wait()



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

def tune_gain(Ku, Tu, mode="clasic"):
    if mode == "clasic":
        # clasic PID
        print("Clasic")
        kp = 0.6 * Ku
        ki = 1.2 * Ku /Tu
        kd = 3 * Ku * Tu / 40
    elif mode == "noovershoot":
        print("No Over Shoot")
        # No overshoot
        kp = Ku / 5
        ki = (2/5)*Ku / Tu
        kd = Ku * Tu / 15
    else:
        print("invaild mdoe")

    print("kp: ", kp, "ki: ", ki , "kd: ", kd)
    return [kp, ki, kd]

import rospy
from simple_pid import PID
from sensor_msgs.msg import JointState
from vision_utils.logger import get_logger
import numpy as np
from std_msgs.msg import Float32
import math

rospy.init_node("PTU_node")
rospy.Subscriber("/joint_states", JointState, state_cb)
rospy.Subscriber("/ref_pan_angle", Float32, angle_cb)
rospy.Subscriber("/ref_tilt_angle", Float32, tilt_angle_cb)

Ku = 9
Tu = 3.6

[kp, ki, kd] = tune_gain(Ku,Tu,"noovershoot")

pid_pan = PID(kp, ki-0.2, kd, setpoint=0)
pid_tilt = PID(0.2, 0, 0, setpoint=0)

v_pan = position[0]*180/np.pi
v_tilt = position[1]*180/np.pi

pid_pan.output_limits = (-167.0, 167.0)
pid_tilt.output_limits = (-29, 90)

x.set_speed_mode()
x.wait()


# rate = rospy.Rate(50) # 50hz
pid_pan.sample_time = 0.02
pid_tilt.sample_time = 0.02
while not rospy.is_shutdown():
    ref = 45
    error = ref - v_pan
    if abs(error) < 3:
        v_pan = ref

    pid_pan.setpoint = ref
    control_pan = pid_pan(v_pan)
    # print(control_pan)

    pid_tilt.setpoint = 0
    control_tilt = pid_tilt(v_tilt)

    x.pan_angle(control_pan)
    v_pan = position[0]*180/np.pi

    x.tilt_angle(control_tilt)
    v_tilt = position[1]*180/np.pi




x.stream.close()

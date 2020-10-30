#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from simple_pid import PID
import numpy as np
from flir_ptu.ptu import PTU
from vision_utils.logger import get_logger
import time
logger = get_logger()

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

x.set_speed_mode()
x.wait()

def tune_gain(Ku, Tu, mode="clasic"):
    if mode == "clasic":
        # clasic PID
        print("Clasic")
        kp = 0.6 * Ku
        ki = 1.2 * Ku / Tu
        kd = 3 * Ku * Tu / 40
    elif mode == "noovershoot":
        print("No Over Shoot")
        # No overshoot
        kp = Ku / 5
        ki = (2/5)*Ku / Tu
        kd = Ku * Tu / 15
    else:
        print("invaild mdoe")

position = [0.0, 0.0]

pan_old_angle = 200
pan_angle = 200
tilt_angle = 200
tilt_old_angle = 200


def state_cb(msg):
    global position
    if msg.name == ["ptu_panner", "ptu_tilter"]:
        position = msg.position


def angle_cb(msg):
    global pan_angle
    pan_angle = msg.data
    # print("pan angle: ", angle)


def tilt_angle_cb(msg):
    global tilt_angle
    tilt_angle = msg.data
    # print("tilt angle: ", tilt_angle)


rospy.init_node("PTU_node")
rospy.Subscriber("/joint_states", JointState, state_cb)
rospy.Subscriber("/ref_pan_angle", Float32, angle_cb)
rospy.Subscriber("/ref_tilt_angle", Float32, tilt_angle_cb)
pub = rospy.Publisher("/angle_ref", Float32, queue_size=1)
pub_tilt = rospy.Publisher("/tilt_angle_ref", Float32, queue_size=1)

pid_pan = PID(400, 54.62, 181.33, setpoint=0)
pid_tilt = PID(150, 0, 0, setpoint=0)

v_pan = position[0]*180/np.pi
v_tilt = position[1]*180/np.pi

# pid_pan.output_limits = (-168.0, 168.0)
# pid_tilt.output_limits = (-30, 90)

rate = rospy.Rate(50) # 50hz
pid_pan.sample_time = 0.01
pid_tilt.sample_time = 0.01

tilt_ref = 200
ref = 200
while not rospy.is_shutdown():
    control_pan = pid_pan(v_pan)
    control_tilt = pid_tilt(v_tilt)
    
    if -180 < pan_angle < 180:
        ref = pan_angle

    if -65 < tilt_angle < 10:
        tilt_ref = tilt_angle + 5

    error = ref - v_pan
    error_tilt = tilt_ref - v_tilt



    # logger.debug(pan_angle)

    if -180 <= pan_angle <= 180:
        
        pan_old_angle = pan_angle
        tilt_old_angle = tilt_angle

        if abs(error) >= 5:
            x.pan_speed(int(control_pan))
        else:
            x.pan_speed(0)

        if abs(error_tilt) >= 5:
            x.tilt_speed(int(control_tilt))
        else:
            x.tilt_speed(0)

        pub.publish(ref)
        pub_tilt.publish(tilt_ref)
    else:
        pan_angle = pan_old_angle
        tilt_angle = tilt_old_angle
        x.pan_speed(0)
        x.tilt_speed(0)
        logger.warning("Invaild angle")


    
    pid_pan.setpoint = pan_angle
    pid_tilt.setpoint = tilt_angle
    
    v_pan = position[0]*180/np.pi   
    v_tilt = position[1]*180/np.pi
    rate.sleep()


x.stream.close()

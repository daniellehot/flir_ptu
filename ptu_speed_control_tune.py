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

    print("kp: ", kp, "ki: ", ki, "kd: ", kd)
    return [kp, ki, kd]

x = PTU("192.168.1.110", 4000, debug=False)
x.connect()

# x.reset()
# x.wait()

x.set_speed_mode()
x.wait()

# set upper speed limit
x.pan_speed_max(11448)
x.wait()

# # read accel
# x.pan_accel()
# x.wait()



# x.tilt_speed(2000)

# x.pan_speed(2000)
# x.wait()


# x.set_position_mode()
# x.wait()

# x.tilt_angle(-25)
# x.wait()

X = np.linspace(-np.pi, np.pi, num=1000)

# numpy sine values
y = np.sin(X) * 180/np.pi




position = [0.0, 0.0]

pan_angle = 200
tilt_angle = 200


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
pub = rospy.Publisher("/control_output", Float32, queue_size=1)

ku = 1100
tu = 5.2

[kp,ki,kd] = tune_gain(ku,tu,"noovershoot")

pid_pan = PID(kp, ki-30, kd-200, setpoint=45)
# pid_tilt = PID(100, 0, 0, setpoint=0)

v_pan = position[0]*180/np.pi
# v_tilt = position[1]*180/np.pi

pid_pan.output_limits = (-11448, 11448)
# pid_tilt.output_limits = (-30, 90)

# rate = rospy.Rate(50) # 50hz
pid_pan.sample_time = 0.02
# pid_tilt.sample_time = 0.02
while not rospy.is_shutdown():

    ref = 45
    error = ref - v_pan
    # print("Error: ", error)
    if abs(error) < 3:
        v_pan = ref
        
    # pid_pan.setpoint = ref
    control_pan = pid_pan(v_pan)
  
    # print("Control: ", control_pan)
    # print("Control: ",int(control_pan))
    # control_tilt = pid_tilt(v_tilt)
    if abs(error) >= 3:
        x.pan_speed(int(control_pan))
        pub.publish(control_pan)
    else:
        x.pan_speed(0)
        pub.publish(0)

    # x.tilt_speed(int(control_tilt))
    
    
    # pid_tilt.setpoint = tilt_angle

    v_pan = position[0]*180/np.pi   
    # v_tilt = position[1]*180/np.pi


x.stream.close()

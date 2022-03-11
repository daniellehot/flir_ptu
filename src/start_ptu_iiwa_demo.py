#!/usr/bin/env python3
from flir_ptu.ptu import PTU
import time
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

position = [70.0, -63.0]

rospy.init_node("ptu_joint_states_node")

x = PTU("192.168.1.110", 4000, debug=False)
x.stream.close()
x.connect()
x.reset()
x.wait()
x.set_position_mode()
x.wait()
x.pan_angle(position[0])
x.tilt_angle(position[1])

pub = rospy.Publisher('ptu_joint_states_demo', JointState, queue_size=1)
joint_msg = JointState()
joint_msg.name = ["ptu_panner", "ptu_tilter"]

rate = rospy.Rate(50) # 50hz
while not rospy.is_shutdown():
    joint_msg.header.stamp = rospy.Time.now()
    pan_angle = x.pan_angle()*(np.pi/180)
    tilt_angle = x.tilt_angle()*(np.pi/180)

    joint_msg.position = [pan_angle,tilt_angle]
    pub.publish(joint_msg)
    rate.sleep()

x.stream.close()

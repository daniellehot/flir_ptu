#!/usr/bin/env python3

from flir_ptu.ptu import PTU
from vision_utils.logger import get_logger
import time
logger =  get_logger()


x = PTU("192.168.1.110", 4000, debug=False)
x.stream.close()
x.connect()


# reset PTU
x.reset()
x.wait()

import numpy as np
# set upper speed limit
x.pan_speed_max(16000)
x.wait()


import rospy
from sensor_msgs.msg import JointState

joint_msg = JointState()
joint_msg.name = ["ptu_panner", "ptu_tilter"]
rospy.init_node("ptu_joint_states_node")
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

rate = rospy.Rate(50) # 50hz
while not rospy.is_shutdown():
    joint_msg.header.stamp = rospy.Time.now()
    pan_angle = x.pan_angle()*(np.pi/180)
    tilt_angle = x.tilt_angle()*(np.pi/180)

    joint_msg.position = [pan_angle,tilt_angle]
    pub.publish(joint_msg)
    rate.sleep()

x.stream.close()

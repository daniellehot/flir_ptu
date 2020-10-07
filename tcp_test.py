#!/usr/bin/env python3

from flir_ptu.ptu import PTU
from vision_utils.logger import get_logger
import time
import rospy
from sensor_msgs.msg import JointState
import numpy as np

logger =  get_logger()

rospy.init_node("ptu_joint_states_node")
pub = rospy.Publisher('joint_states', JointState, queue_size=1)

x = PTU("192.168.1.110", 4000, debug=False)
x.connect()

# reset PTU
x.reset()
x.wait()

# set upper speed limit
x.pan_speed_max()
x.pan_speed_max(8000)
x.wait()

# read accel
x.pan_accel()
x.wait()

x.pan(1000)
x.wait()

joint_msg = JointState()
joint_msg.name = ["ptu_panner", "ptu_tilter"]

rate = rospy.Rate(50) # 50hz
while not rospy.is_shutdown():
    joint_msg.header.stamp = rospy.Time.now()

    pan_angle = x.pan_angle()*(np.pi/180)
    tilt_angle = x.tilt_angle()*(np.pi/180)

    joint_msg.position = [pan_angle,tilt_angle]
    pub.publish(joint_msg)
    # hello_str = "hello world %s" % rospy.get_time()
    # rospy.loginfo(hello_str)
    # pub.publish(hello_str)
    rate.sleep()

x.stream.close()

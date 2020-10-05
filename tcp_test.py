#!/usr/bin/env python3

from flir_ptu.ptu import PTU
from vision_utils.logger import get_logger
import time
import rospy
from sensor_msgs.msg import JointState
logger =  get_logger()

rospy.init_node("PTU_node")
pub = rospy.Publisher('joint_states', JointState, queue_size=1)

x = PTU("192.168.0.110", 4000, debug=False)
x.connect()

# reset PTU
x.reset()
x.wait()

# having fun with sines
import numpy as np
sin = np.arcsin(np.arange(-1,1.0,0.01)) * 180/np.pi
sin_inv = np.arcsin(np.arange(1.0,-1,0.01)) * 180/np.pi

# set upper speed limit
x.pan_speed_max()
x.pan_speed_max(8000)
x.wait()

# read accel
x.pan_accel()
x.wait()

joint_msg = JointState()
joint_msg.name = ["ptu_panner", "ptu_tilter"]

rate = rospy.Rate(50) # 10hz
while not rospy.is_shutdown():
    joint_msg.header.stamp = rospy.Time()
    joint_msg.position = [x.pan_angle(),x.tilt_angle()]
    pub.publish(joint_msg)
    # hello_str = "hello world %s" % rospy.get_time()
    # rospy.loginfo(hello_str)
    # pub.publish(hello_str)
    rate.sleep()

x.stream.close()

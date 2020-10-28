#!/usr/bin/env python3
import numpy as np
from flir_ptu.ptu import PTU
from vision_utils.logger import get_logger
logger =  get_logger()

x = PTU("192.168.1.110", 4000, debug=False)
x.connect()

# x.reset()
# x.wait()

# set upper speed limit
x.pan_speed_max(11000)
x.wait()

x.set_position_mode()

x.pan_speed(2000)
x.wait()

x.tilt_speed(2000)
x.wait()

position = [0.0, 0.0]

angle = 200
tilt_angle = 200

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


rospy.init_node("PTU_node")
rospy.Subscriber("/joint_states", JointState, state_cb)
rospy.Subscriber("/ref_pan_angle", Float32, angle_cb)
rospy.Subscriber("/ref_tilt_angle", Float32, tilt_angle_cb)
pub = rospy.Publisher("/angle_ref", Float32, queue_size=1)
pub_tilt = rospy.Publisher("/tilt_angle_ref", Float32, queue_size=1)


v_pan = position[0]*180/np.pi
v_tilt = position[1]*180/np.pi

ref = 0
tilt_ref = 0

rate = rospy.Rate(50)
while not rospy.is_shutdown():
    
    #If a person is detected and is within the joint limits, move pan joint 
    if -180 < angle < 180:
        ref = angle
        
        #Calculate the error and move the joint if its outside the threshold
        error = ref - v_pan
        if abs(error) >= 5:
            x.pan_angle(ref)
    
    #If a person is detected and is within the joint limits, move tilt joint
    if -65 < tilt_angle < 10:
        #Add an offset on the tilt so more of the human is in the field of view of the camera
        tilt_ref = tilt_angle +5

        #Calculate the error and move the joint if its outside the threshold
        error_tilt = tilt_ref - v_tilt
        if abs(error_tilt) >= 3:
            x.tilt_angle(tilt_ref)

    #Publish the ref input to the PTU
    pub.publish(ref)
    pub_tilt.publish(tilt_ref)

    #Read the newest joint angle position and convert it to degress    
    v_pan = position[0]*180/np.pi
    v_tilt = position[1]*180/np.pi
    rate.sleep()



x.stream.close()

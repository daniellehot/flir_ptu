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
    # print("pan angle: ", angle)

def tilt_angle_cb(msg):
    global tilt_angle
    tilt_angle = msg.data
    # print("tilt angle: ", tilt_angle)



import rospy
# from simple_pid import PID
from sensor_msgs.msg import JointState
from vision_utils.logger import get_logger
import numpy as np
from std_msgs.msg import Float32, Bool


rospy.init_node("PTU_node")
rospy.Subscriber("/joint_states", JointState, state_cb)
rospy.Subscriber("/ref_pan_angle", Float32, angle_cb)
rospy.Subscriber("/ref_tilt_angle", Float32, tilt_angle_cb)
pub = rospy.Publisher("/angle_ref", Float32, queue_size=1)
pub_tilt = rospy.Publisher("/tilt_angle_ref", Float32, queue_size=1)
not_move_pub = rospy.Publisher("/ptu_not_moving", Bool, queue_size=1)
noperson_pub = rospy.Publisher("/noperson", Bool, queue_size=1)


v_pan = position[0]*180/np.pi
v_tilt = position[1]*180/np.pi

ref = 0
tilt_ref = 0
time_old = time.time()
rate = rospy.Rate(50)
first = True
speed_first = True

last_human_angle = 0
person_not_detected = 200
moving_left= True
moving_right = True
tilt_first = True
time_before_search = 2

MAX_PAN = 30
MIN_PAN = -90

pan_tolerance = 10
tilt_tolerance = 5

while not rospy.is_shutdown():


    if angle < person_not_detected:
        #If a person is detected and is within the joint limits, move pan joint
        last_human_angle = angle
        angle = max(MIN_PAN, angle)
        angle = min(MAX_PAN, angle)

        ref = angle
        #Calculate the error and move the joint if its outside the threshold
        error = ref - v_pan
        if abs(error) >= pan_tolerance:
            x.pan_angle(ref)

    if tilt_angle < person_not_detected:
    #If a person is detected and is within the joint limits, move tilt joint
        tilt_angle = max(-28, tilt_angle)
        tilt_angle = min(10, tilt_angle)
        tilt_ref = tilt_angle
        #Calculate the error and move the joint if its outside the threshold
        error_tilt = tilt_ref - v_tilt
        if abs(error_tilt) >= tilt_tolerance:
            x.tilt_angle(tilt_ref)

    #Publish the ref input to the PTU
    pub.publish(ref)
    pub_tilt.publish(tilt_ref)


    # #search for a human if a human was not found.
    if angle == person_not_detected and tilt_angle == person_not_detected:
        if first:
            time_old = time.time()
            first = False
            speed_first = True
            print("starting timer for human scanning")
            x.pan_speed(1000)
            x.wait()
        if time.time() - time_old > time_before_search:
            noperson_pub.publish(True)
            if tilt_first:
                x.tilt_angle(-10)
                tilt_first = False
            if  last_human_angle < 0:
                ref = max(-45,MIN_PAN)
                error = ref - v_pan
                if abs(error) >= tilt_tolerance:
                    if moving_left:
                        print("moving left")
                        x.pan_angle(ref)
                        moving_left = False
                else:
                    moving_left = True
                    last_human_angle = min(45,MAX_PAN)
            else:
                ref = MAX_PAN
                error = ref - v_pan
                if abs(error) >= pan_tolerance:
                    if moving_right:
                        print("moving right")
                        x.pan_angle(ref)
                        moving_right = False
                else:
                    moving_right = True
                    last_human_angle = max(-45,MIN_PAN)
    else:
        noperson_pub.publish(False)
        first = True
        moving_right = True
        moving_left = True
        tilt_first = True
        if speed_first:
            x.pan_speed(2000)
            x.wait()
            speed_first = False


    #Read the newest joint angle position and convert it to degress
    v_pan_old = v_pan
    v_tilt_old = v_tilt

    v_pan = position[0]*180/np.pi
    v_tilt = position[1]*180/np.pi

    if v_pan_old == v_pan and v_tilt_old == v_tilt:
        not_move_pub.publish(True)
    else:
        not_move_pub.publish(False)

    rate.sleep()



x.stream.close()

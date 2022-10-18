#!/usr/bin/env python

import pickle
import rospy
import mavros
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *


infile = open('/dev/shm/center.pkl', 'rb')
cntr = None

def get_center():
    global cntr
    try:
        cntr = pickle.load(infile)
    except:
        pass


# callback method for state sub
mavros.set_namespace()
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

# callback method for pose sub
current_pose = PoseStamped()
def pose_cb(pose):
    global current_pose
    current_pose = pose

vel_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, queue_size=10)
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, pose_cb)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

proximity = 1
h = 1
setpoints = np.loadtxt(open('/home/pravardhan/test_ws/src/pixhawk/src/track and center/loc.csv', 'rb'), delimiter=',')
# setpoints = [(0,0,h), (5,0,h), (5,5,h), (0,5,h), (0,0,h)]

pose = PoseStamped()
pose.pose.position.x = setpoints[0][0]
pose.pose.position.y = setpoints[0][1]
pose.pose.position.z = h

vel = TwistStamped()

def is_near(desired_pose):
    current = np.array((current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z))
    desired = np.array((desired_pose.pose.position.x,
                        desired_pose.pose.position.y,
                        desired_pose.pose.position.z))
    return np.linalg.norm(current - desired) < proximity


def position_control():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # rospy.wait_for_service("/mavros/cmd/arming")
    # arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    # rospy.wait_for_service("/mavros/set_mode")
    # set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # wait for FCU connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    height = 0
    for i in range(10):
        height += current_pose.pose.position.z
        rate.sleep()
    h = height / 10 # take average

    last_req = rospy.Time.now()

    centering = False
    
    point = 0
    while(not rospy.is_shutdown()):
        if not current_state.armed:
            break
        
        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()

        if(current_state.mode == "OFFBOARD" and is_near(pose)):
            point += 1
            x,y = setpoints[point % len(setpoints)]
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = h
            centering = True

        get_center()
        r, c, cx, cy, area = cntr
        if centering and r:
            if np.linalg.norm((r-cx, c - cy)) < 10:
                centering = False
                continue

            vel.header.stamp = rospy.Time.now()
            vel.twist.linear.y = max(min((cx - r) / 100 , 0.3), -0.5)
            vel.twist.linear.x = max(min((cy - c) / 100 , 0.3), -0.5)

            vel_pub.publish(vel)
            
        else:
            local_pos_pub.publish(pose)

        rate.sleep()
    
    infile.close()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
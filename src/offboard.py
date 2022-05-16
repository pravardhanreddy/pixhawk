#!/usr/bin/env python

import rospy
import mavros
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

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

local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, pose_cb)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

height = 1
proximity = 0.1
setpoints = [(0,0,height), (1,0,height)]

pose = PoseStamped()
pose.pose.position.x = setpoints[0][0]
pose.pose.position.y = setpoints[0][1]
pose.pose.position.z = setpoints[0][2]

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

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected and not current_state.armed:
        rate.sleep()

    # arm the drone
    # arming_client(True)
    rospy.loginfo("Vehicle armed: %r" % current_state.armed)


    last_request = rospy.get_rostime()
    point = 0
    while not rospy.is_shutdown():

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
            x,y,z = setpoints[point % len(setpoints)]
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
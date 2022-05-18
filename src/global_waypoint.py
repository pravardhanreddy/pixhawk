#!/usr/bin/env python

import rospy
import mavros
import numpy as np
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import quaternion_from_euler
from geopy.distance import distance
from pygeodesy.geoids import GeoidPGM

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

# conversion between AMSL (above mean sea level) and ellipsoid height
def geoid_height(lat, lon):
    return _egm96.height(lat, lon)

# callback method for state sub
mavros.set_namespace()
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

# callback method for pose sub
current_pose = NavSatFix()
def pose_cb(pose):
    global current_pose
    current_pose = pose

global_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'global'), GeoPoseStamped, queue_size=10)
global_pos_sub = rospy.Subscriber(mavros.get_topic('global_position', 'global'), NavSatFix, pose_cb)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

proximity = 0.1
setpoints = np.loadtxt(open('/home/pravardhan/test_ws/src/test_pkg/src/global.csv', 'rb'), delimiter=',', skiprows=1)

pose = GeoPoseStamped()

def is_near(desired_pose):
    c1 = (desired_pose.pose.position.latitude, desired_pose.pose.position.longitude)
    c2 = (current_pose.latitude, current_pose.longitude)
    # print(f'distance between {c1} and {c2} is { distance(c1,c2).km * 1000 }')
    return distance(c1,c2).km * 1000 < proximity # convert km to meters


def position_control():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        global_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected or not current_state.armed:
        rate.sleep()

    height = 0
    for i in range(10):
        height += current_pose.altitude
        rate.sleep()
    height = height / 10 # take average

    # arm the drone
    # arming_client(True)
    rospy.loginfo(f"Vehicle armed: {current_state.armed}, at height {height:.2f}m")


    point = 0
    pose.pose.position.latitude = setpoints[point][0]
    pose.pose.position.longitude = setpoints[point][1]
    pose.pose.position.altitude = setpoints[point][2] + height - geoid_height(setpoints[point][0], setpoints[point][1])

    q = quaternion_from_euler(0,0,setpoints[point][3])
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    while not rospy.is_shutdown():

        if not current_state.armed:
            break
        
        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()

        if(current_state.mode == "OFFBOARD" and is_near(pose) and point < len(setpoints)):
            pose.pose.position.latitude = setpoints[point][0]
            pose.pose.position.longitude = setpoints[point][1]
            pose.pose.position.altitude = setpoints[point][2] + height - geoid_height(setpoints[point][0], setpoints[point][1])
            print('Height setpoint:', pose.pose.position.altitude)

            q = quaternion_from_euler(0,0,setpoints[point][3])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            point += 1


        global_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
import mavros
from mavros_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
from mavros_msgs.srv import *

SPEED = 1
IMAGE_WIDTH = 1280
FLIP = -1 # 1: No Flip, -1: Horizontal Flip (If the image is like a reflection or not)

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

# callback method for detection sub
center = 0
def detection_cb(msg: Detection2DArray):
    global center
    if(len(msg.detections) == 0):
        return
    if(len(msg.detections[0].results) == 0):
        return
    id = msg.detections[0].results[0].id
    x = msg.detections[0].bbox.center.x
    center = FLIP * (x/IMAGE_WIDTH - 0.5) # Normalised
    center = center if id == 1 else 0.
    print(f'id: {id}, center: {center:.2f}')

vel_pub = rospy.Publisher(mavros.get_topic('setpoint_raw', 'local'), PositionTarget, queue_size=10)
local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, pose_cb)
detection_sub = rospy.Subscriber('/detectnet/detections', Detection2DArray, detection_cb)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 


vel = PositionTarget()
vel.coordinate_frame = 1 # MAV_FRAME_LOCAL_NED
vel.type_mask = 0b0000010111111000 # Only use position and yaw_rate
# vel.position.z = 1.5
vel.position.z = current_pose.pose.position.z

def position_control():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    for i in range(100):
        vel_pub.publish(vel)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected or not current_state.armed:
        rate.sleep()
    
    height = 0
    for i in range(10):
        height += current_pose.pose.position.z
        rate.sleep()
    height = height / 10 # take average
    vel.position.z = height

    # arm the drone
    # arming_client(True)
    rospy.loginfo("Vehicle armed: %r" % current_state.armed)


    while not rospy.is_shutdown():

        if not current_state.armed:
            break
        
        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        vel.header.stamp = rospy.Time.now()

        vel.yaw_rate = center * SPEED

        vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
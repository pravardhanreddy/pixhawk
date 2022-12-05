#!/usr/bin/env python

import rospy
import mavros
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import cv2

vid = cv2.VideoCapture(42)

center = (320, 180)
frame_area = 230400
upper_area = frame_area * 0.3
lower_area = frame_area * 0.15

def get_center(vid):
    ret, img = vid.read()
    cv2.circle(img, (320, 180), 3, (0,0,255), -1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 19)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    thresh = cv2.threshold(sharpen,100,255, cv2.THRESH_BINARY_INV)[1]
    # cv2.imshow('thresh', thresh)


    # apply morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
    clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17,17))
    clean1 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    blur = cv2.medianBlur(clean1, 19)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    clean = cv2.threshold(sharpen,100,255, cv2.THRESH_BINARY_INV)[1]
    # cv2.imshow('clean', clean)
    contours = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    rot_rect = None
    area = None

    if len(contours) == 0:
        return None, None, None

    for c in contours:
        temp_rot_rect = cv2.minAreaRect(c)
        temp_area = cv2.contourArea(c)
        if area == None or temp_area > area:
            area = temp_area
            rot_rect = temp_rot_rect

    box = cv2.boxPoints(rot_rect)
    box = np.int0(box)
    # draw rotated rectangle on copy of img
    cv2.drawContours(img,[box],0,(0,0,0),2)
    cv2.circle(img, np.int0(rot_rect[0]), 3, (255,0,0), -1)
    cv2.putText(img, str(np.linalg.norm((rot_rect[0][0]-center[0], rot_rect[0][1] - center[1]))), np.int0(rot_rect[0]), cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, (0,255,0), 2, cv2.LINE_AA)

    cv2.imshow('shape', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        pass
    if rot_rect:
        r,c = np.int0(rot_rect[0])
        return r, c, area

    return None, None, None

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
h = 3
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

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # wait for FCU connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    centering = False
    
    point = 0
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
        
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

        r,c,area = get_center(vid)
        if centering:
            vel = TwistStamped()
            vel.header.stamp = rospy.Time.now()

            if r:
                if np.linalg.norm((r-center[0], c - center[1])) < 10 and lower_area < area < upper_area:
                    centering = False
                    continue

                vel.twist.linear.y = max(min((center[0] - r) / 100 , 0.3), -0.5)
                vel.twist.linear.x = max(min((center[1] - c) / 100 , 0.3), -0.5)
                if area < lower_area:
                    vel.twist.linear.z = -0.1
                elif area > upper_area:
                    vel.twist.linear.z = 0.1
            

            vel_pub.publish(vel)
            
        else:
            local_pos_pub.publish(pose)

        rate.sleep()

    vid.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
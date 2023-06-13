#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import std_msgs.msg as msg
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
import json
import math
import time

vel_linear = 0
vel_angular = 0
neck_angle = -45

status_nav = 0
status_nav_old = 0

x_cmd = 0
y_cmd = 0
z_cmd = 0
w_cmd = 0

map_data = ""
map_data_old = ""
stop_cmd = False

timeNow = 0
timeOld = 0

def receiveVel(data):
    global vel_linear, vel_angular, timeOld, timeNow
    x = json.loads(data.data)
    vel_linear = x[0]["linear"]
    vel_angular = x[0]["angular"]
    timeOld = round(time.time() * 1000)

def receiveNeck(data):
    global neck_angle, neck
    x = json.loads(data.data)
    neck_angle = int(x[0]["angle"])
    print(neck_angle)
    neck.publish(neck_angle*1000)

def receiveCoord(data):
    global map_data
    msg_map = {
        "position": {
            "x": data.pose.pose.position.x,
            "y": data.pose.pose.position.y
        },
        "orientation":{
            "z": data.pose.pose.orientation.z,
            "w": data.pose.pose.orientation.w
        }
    }
    map_data = json.dumps(msg_map)

def receiveMap(data):
    global x_cmd, y_cmd, z_cmd, w_cmd, nav_start, nav, status_nav
    dataJson = json.loads(data.data)
    angle = dataJson["angle"]
    x_cmd = dataJson["x"]
    y_cmd = dataJson["y"]

    # CALCULATE QUATERNIONS
    qX = math.sin(angle/2) * 0
    qY = math.sin(angle/2) * 0
    qZ = math.sin(angle/2) * 1
    qW = math.cos(angle/2)

    nav.header.frame_id = 'map'
    nav.pose.position.x = x_cmd
    nav.pose.position.y = y_cmd
    nav.pose.orientation.z = qZ
    nav.pose.orientation.w = qW
    if(status_nav != 1):
        nav_start.publish(nav)

def receiveCancel(data):
    global stop_cmd, nav_stop
    dataJson = json.loads(data.data)
    stop_cmd = dataJson["command"]
    stop = GoalID()
    nav_stop.publish(stop)



def receiveStatusNav(data):
    # Translate goal status from 
    # http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
    global status_nav, ohmni_status, status_nav_old
    if(len(data.status_list) > 0):
        count = len(data.status_list) - 1
        status_nav = data.status_list[count].status
        if(status_nav != status_nav_old):
            status_nav_old = status_nav
            statusData = {
                "status": status_nav
            }
            ohmni_status.publish(json.dumps(statusData))


def ohmni_bridge():
    global vel_linear, vel_angular, map_data, map_data_old, stop_cmd
    global x_cmd, y_cmd, z_cmd, w_cmd, neck
    global status_nav, nav_test, nav, nav_start, nav_stop
    global timeOld, timeNow
    global ohmni_status

    # Publish Data
    rospy.init_node('Client_Node', anonymous=True)
    pub = rospy.Publisher('/tb_cmd_vel',Twist, queue_size=10)
    neck = rospy.Publisher('/tb_cmd_new_neck',msg.Int32, queue_size=10)
    ohmni_map = rospy.Publisher('/ohmni_map/position',msg.String, queue_size=10)
    ohmni_status = rospy.Publisher('/ohmni_map/status',msg.String, queue_size=10)

    # Navigation
    nav_start = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    nav_test = rospy.Publisher('/navigation/goal', PoseStamped, queue_size=10)
    nav_stop = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

    nav = PoseStamped()
    vel = Twist()

    rospy.Subscriber('ohmni_move/cmd_vel', msg.String, receiveVel)
    rospy.Subscriber('ohmni_neck/neck_move', msg.String, receiveNeck)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, receiveCoord)
    rospy.Subscriber('/move_base/status', GoalStatusArray, receiveStatusNav)

    # OHMNI MAP
    rospy.Subscriber('/ohmni_goal/coord', msg.String, receiveMap)
    rospy.Subscriber('/ohmni_goal/cancel', msg.String, receiveCancel)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Publish CMD VEL
        if(status_nav != 1):
            timeNow = round(time.time() * 1000)
            vel.linear.x = vel_linear 
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = vel_angular
            if(timeNow - timeOld >= 1000):
                timeOld = timeNow
                vel_linear = 0
                vel_angular = 0
            else:
                pub.publish(vel)

            neck.publish(neck_angle*1000)
            if(map_data_old != map_data):
                map_data_old = map_data
                ohmni_map.publish(map_data)
 
        rate.sleep()

if __name__ == "__main__":
    try:
        ohmni_bridge()
    except rospy.ROSInterruptException:
        pass

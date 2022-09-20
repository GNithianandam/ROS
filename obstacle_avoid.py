#!/usr/bin/env python3
# BEGIN ALL

import math
import rospy
from geometry_msgs.msg import Twist,PoseStamped
from rospy.timer import sleep 
from sensor_msgs.msg import LaserScan,Range,Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



def f1_callback(msg):
    global fl
    fl=msg.range
    
    
def fr_callback(msg):
    global fr
    fr=msg.range
    
   
def lidar_callback(msg):
    global regions
    regions= {'right':min(min(msg.ranges[539:610]),12),       #right
    'fright':min(min(msg.ranges[611:684]),12),                #fright
    'front':min(min(msg.ranges[0:35]),12),                    #front1
    'front2':min(min(msg.ranges[683:719]),12),                #front2
    'fleft':min(min(msg.ranges[36:107]),12),                  #fleft
    'left':min(min(msg.ranges[108:179]),12)}                  #left
    navigate()
    rotate()
    
    
def IMU_callback(msg):
    global theta 
    global theta_deg
    global quadrant
    roll=0
    pitch=0
    orientation_q=msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, theta) = euler_from_quaternion (orientation_list)
    theta_deg= math.degrees(theta)
    
def navigate(): 
    if regions['front'] > 0.3 and regions['fright'] > 0.3 and regions['front2']>0.3 and regions['fleft'] > 0.3 and fl>0.3 and fr>0.3:                                                               #front
        linear_x = 0.2
        angular_z = 0    
    elif regions['left'] > 0.3 :         #left
        linear_x = 0
        angular_z = 0.4
        
    elif ( regions['right'] >0.3) :        #right
        linear_x = 0
        angular_z =-0.4
        
    else:                                                                                      #back
        linear_x=-0.1
        angular_z=0

    move.linear.x=linear_x
    move.angular.z=angular_z
    
    

def rotate():
    if regions['front'] > 1 and regions['front2']>1 and regions['fleft'] > 0.3 and regions['left']>0.3 and regions['fright'] > 0.3 and regions['right']>0.3 and ((theta_deg<175 and theta_deg>0)or (theta_deg>-175 and theta_deg<0)):  
        if theta_deg>0:
                move.linear.x=0.0
                move.angular.z=0.2
        else:
                move.linear.x=0.0
                move.angular.z=-0.2
                
    pub.publish(move)
    
   


rospy.init_node('Obstacle_avoid')
sub=rospy.Subscriber('range/fl',Range,f1_callback)
sub2=rospy.Subscriber('range/fr',Range,fr_callback)
sub3=rospy.Subscriber('scan',LaserScan,lidar_callback)
sub4=rospy.Subscriber('/imu',Imu,IMU_callback)
pub= rospy.Publisher('/cmd_vel',Twist, queue_size=1) 
move=Twist()
rospy.Rate(0.7)
rospy.spin()


        
        
        
        
   
    

     
   

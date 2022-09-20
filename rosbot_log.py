#!/usr/bin/env python3
# BEGIN ALL
import math
import time
import rospy
from geometry_msgs.msg import Twist,PoseStamped
from rospy.timer import sleep 
from sensor_msgs.msg import LaserScan,Range,Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class distancetravel:   
    def __init__(self):
        self.firstrun=True
        self.distance=0
        self.old_x=0
        self.old_y=0
        self.startT=time.perf_counter()

    def listen(self): 
        rospy.init_node('ROSbot_LogInfo')
        self.sub=rospy.Subscriber('/odom',Odometry,self.odometryCb)
        rospy.Rate(0.2)
        rospy.spin()

    def odometryCb(self,msg):
        if self.firstrun:
            self.old_x=msg.pose.pose.position.x
            self.old_y=msg.pose.pose.position.y
        x= msg.pose.pose.position.x
        y= msg.pose.pose.position.y
        d_increment = math.sqrt(((x - self.old_x) * (x - self.old_x) )+
                   ((y - self.old_y)*(y - self.old_y)))
        self.distance = self.distance + d_increment
        self.old_x=x
        self.old_y=y
        self.firstrun=False
        currentT=time.perf_counter()
        TotalT=(currentT-self.startT)
        rospy.loginfo(f'Distance= {self.distance} meters')
        rospy.loginfo(f'Time= {TotalT:0.4f} seconds')
        

if __name__ == '__main__':
    odom = distancetravel()
    odom.listen()
    
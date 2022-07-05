#!/usr/bin/env python 
# coding: utf-8

from Wifibot import wifibot
from math import sqrt , pi
import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import time

class deplacements:
    def __init__(self):
        self.Robot = wifibot()
        print("Deplacement 1")
        self.x1 = 0
        self.x2 = 0
        self.y1 = 0
        self.y2 = 0
        self.z1 = 0
        self.z2 = 0
        self.w1 = 0
        self.w2 = 0
        self.angleRobot = 0
        self.angleDestination = 0 
        self.go = False
        self.orientation = True
        self.Goall = self.Goal()
        self.Pose = self.PositionRobot()
        self.arretContinue()

    def arretContinue(self):
        while True:
            if(self.go == False):
                self.Robot.arret()
                time.sleep(0.1)
            else:
                None
        
    def Calcul(self):
        while(self.go == True):
            while(self.orientation == False):
                while ( abs(self.angleDestination - self.angleRobot) > 4 ):#modifier condition
                    if( ((self.angleRobot - self.angleDestination) >= 180 and (self.angleRobot - self.angleDestination) < 360) 
                    or ((self.angleRobot - self.angleDestination) > -180 and (self.angleRobot - self.angleDestination) < 0 ) ):
                        self.Robot.gauche()
                    else :
                        self.Robot.droite()
                self.orientation = True
            while( abs(self.x2 - self.x1) > 0.07 and abs(self.y2 - self.y1) > 0.07 ):
                    self.Robot.avancer()
            self.go = False
        
    def Goal(self):
        my_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callbackGoal)

    def PositionRobot(self):
        my_subscriber2 = rospy.Subscriber("/slamware_ros_sdk_server_node/odom", Odometry, self.callbackOdom)
    
    def callbackGoal(self, data):
        self.x2 = data.pose.position.x
        self.y2 = data.pose.position.y
        self.z2 = data.pose.orientation.z
        self.w2 = data.pose.orientation.w
        self.orientation = False
        self.go = True
        self.distance()
        self.Calcul()

    def callbackOdom(self, data):
        self.x1 = data.pose.pose.position.x
        self.y1 = data.pose.pose.position.y
        self.z1 = data.pose.pose.orientation.z
        self.w1 = data.pose.pose.orientation.w
        if(self.z1 >= 0):
            self.angleRobot = ( 180*2*np.arccos(self.w1) ) / pi 
        elif(self.z1 < 0 ):
            self.angleRobot = (180*2*np.arccos(-0.5)) / pi + (180*2*np.arccos(0.5)) / pi - (180*2*np.arccos(self.w1)) / pi 

    def distance(self):
        distance = sqrt((self.x2-self.x1)*(self.x2-self.x1) + (self.y2-self.y1)*(self.y2-self.y1))
        angleX = (self.x2 - self.x1) / distance
        angleY = (self.y2 - self.y1) / distance
        if(angleY > 0):
            self.angleDestination = 180*(np.arccos(angleX)) / pi
        elif(angleY <= 0 ):
            self.angleDestination = 180*(np.arccos(-1) + (np.arccos(-1)-np.arccos(angleX))) / pi
        print(self.angleDestination)
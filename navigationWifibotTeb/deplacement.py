#!/usr/bin/env python
# coding: utf-8

import threading
from Wifibot import wifibot
from math import sqrt , pi
import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

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
        self.leChemin = True
        self.Goall = self.Goal()
        self.chemin = self.Path()
        self.Pose = self.PositionRobot()

        self.tcpThread = threading.Thread(target=self.arretContinue)
        self.tcpThread.deamon = True
        self.tcpThread.start()

    def arretContinue(self):
        while True:
            if(self.go == False):
                self.Robot.arret()
                print("arret")
            else:
                None

    def Calcul(self):
        while(self.go == True):
            while(self.orientation == False):
                while ( abs(self.angleDestination - self.angleRobot) > 4 ): #modifier condition
                    if( ((self.angleRobot - self.angleDestination) >= 180 and (self.angleRobot - self.angleDestination) < 360) or ((self.angleRobot - self.angleDestination) > -180 and (self.angleRobot - self.angleDestination) < 0 ) ):
                        self.Robot.gauche()
                    else: 
                        self.Robot.droite()
                self.orientation = True
            while( abs(self.x2 - self.x1) > 0.05 and abs(self.y2 - self.y1) > 0.05 ):
                    self.Robot.avancer()
            self.go = False

    def Goal(self):
        my_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callbackGoal)

    def PositionRobot(self):
        my_subscriber2 = rospy.Subscriber("/slamware_ros_sdk_server_node/odom", Odometry, self.callbackOdom)

    def Path(self):
        my_subscriber3 = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.callbackPath)

    def callbackOdom(self, data):
        self.x1 = data.pose.pose.position.x
        self.y1 = data.pose.pose.position.y
        self.z1 = data.pose.pose.orientation.z
        self.w1 = data.pose.pose.orientation.w
        if(self.z1 >= 0):
            self.angleRobot = ( 180*2*np.arccos(self.w1) ) / pi
        elif(self.z1 < 0):
            self.angleRobot = ( 180*2*np.arccos(-0.5)) / pi + (180*2*np.arccos(0.5)) / pi - (180*2*np.arccos(self.w1)) / pi 

    def callbackGoal(self, data):
        self.leChemin = True

    def callbackPath(self, data):
        if(self.leChemin == True):
            toutLesPoints = data.poses
            listeTtLesPointsFinal = []

            index = 1
            for point in toutLesPoints :
                if( len(toutLesPoints) > 20) :
                    if(index%8 == 0):
                        listeTtLesPointsFinal.append(point)
                    index += 1
                else :
                    listeTtLesPointsFinal.append(point)
            listeTtLesPointsFinal.append(toutLesPoints[-1])
            print("taille du path : "+str(len(listeTtLesPointsFinal)))

            self.leChemin = False
            i = 1
            for point in listeTtLesPointsFinal:
                print("point : "+str(i))
                self.x2 = point.pose.position.x
                self.y2 = point.pose.position.y
                self.z2 = point.pose.orientation.z
                self.w2 = point.pose.orientation.w
                self.orientation = False
                self.go = True
                self.distance()
                self.Calcul()
                i+=1

    def distance(self):
        distance = sqrt((self.x2-self.x1)*(self.x2-self.x1) + (self.y2-self.y1)*(self.y2-self.y1))
        angleX = (self.x2 - self.x1) / distance
        angleY = (self.y2 - self.y1) / distance
        self.anglex = angleX
        self.angley = angleY
        if(angleY > 0):
            self.angleDestination = 180*(np.arccos(angleX)) / pi
        elif(angleY <= 0 ):
            self.angleDestination = 180*(np.arccos(-1) + (np.arccos(-1)-np.arccos(angleX))) / pi
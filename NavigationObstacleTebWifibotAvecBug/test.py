#!/usr/bin/env python
# coding: utf-8

from nav_msgs.msg import Path
import threading
import rospy
import time

class test:
    def __init__(self):
        self.valide = True
        self.Path()

        self.tcpThread = threading.Thread(target=self.temps)
        self.tcpThread.deamon = True
        self.tcpThread.start()

        self.tcpThread2 = threading.Thread(target=self.un)
        self.tcpThread2.deamon = True
        self.tcpThread2.start()

    def callbackPath(self,data):
        if(self.valide == True):
            liste = data.poses
            print(len(liste))
            print(self.valide)
            print("------------------------------")

    def Path(self):
        my_subscriber3 = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.callbackPath)

    def temps(self):
        start = time.time()
        while(time.time() - start < 5):
            self.valide = True 
        self.valide = False

    def un(self):
        while(True):
            print("1")
            time.sleep(1)
       

if __name__=="__main__":
    rospy.init_node("my_subscriber_node")
    test()
    rospy.spin()
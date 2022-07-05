from deplacement import deplacements
import rospy

if __name__=="__main__":
    rospy.init_node("deplacement_node")
    D1 = deplacements()
    rospy.spin()
#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

# vllt argument für remote controll?!
class HectorNode(object):
    def __init__(self):

        rospy.init_node('hecotr_node')

        self.odometry = Odometry()
        rospy.Subscriber("/ground_truth/state", Odometry , self.pose_callback) # topic published with 100 Hz

        rospy.Subscriber("/sonar_height", Range , self.sonar_callback) # 10 Hz

        # cmd vel innerhalb der drohne erstmal raus lassen
        # self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    
    def pose_callback(self, data):

        # save actual drone position (x,y,z [m]) in global Odometry variable
        self.odometry.pose.pose.position.x = round(data.pose.pose.position.x, 2)
        self.odometry.pose.pose.position.y = round(data.pose.pose.position.y, 2)
        self.odometry.pose.pose.position.z = round(data.pose.pose.position.z, 2)

        #self.odometry.pose.pose.position.x = "{0:.2f}".format(data.pose.pose.position.x)

    def sonar_callback(self, data):
        m  = round(data.range, 2)
        print(self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, m)


if __name__ == '__main__':

    try:
        HectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")
#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import Range
import numpy as np
import math

# vllt argument für remote controll?!
class HectorNode(object):
    def __init__(self):

        self.sonar_offset = 0.17 # m 
        self.drone_z_offset = 0.28 # m

        rospy.init_node('hector_node')

        self.odometry = Odometry()
        rospy.Subscriber("/ground_truth/state", Odometry, self.pose_callback) # topic published with 100 Hz

        rospy.Subscriber("/sonar_height", Range , self.sonar_callback) # 10 Hz
        
        rospy.Subscriber("/release", Empty, self.release_callback)

        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist)
        #self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # read CSV -> Eckpunkte erfasst
        # getSektorForEckpunkte()
        # self.spalten = []
        # self.flypoints = []
        # spalten als self.spalten
        # for spalte in self.spalten:
            # first_fly_point_y = spalten[0] - 5
            # last_fly_point_y = spalten[-1] + 5
            # self.flypoints.append([[first_fly_point_x, first_fly_point_y], [last_fly_point_x, last_fly_point_y]])

        # fake flypoints 
        self.flypoints = [[0,0,10],[20,0],[20,20],[0,20]]

        # flypoints müssen aus den segmentmittelpunkten erstellt werden



    
    def release_callback(self, data):

        # release drone with empty pub on '/release' - topic
            
        for point in self.flypoints:
            self.flyToPosition(point)

        
        # hier muss 
        
        # for i, spalte in enumerate(self.spalten):
        #     spaltenanfang = self.flypoints[i][0]
        #     spaltenende = self.flypoints[i][1]
        #     self.currentSpalte = spalte
        #     self.currentX = spalte[0][0]
        #     self.currentYs = spalte.filter(...)

        #     self.flyToPosition(spaltenanfang)
        #     self.heightsForSpalte = []
        #     self.flyToPosition(spaltenende)
        #     self.heights.append(self.heightsForSpalte)

        return

    def flyToPosition(self, point, tol = 0.2, p_x = 0.2, p_y = 0.2, p_z = 0.2):
        
        # get x and y value [m] from goal position
        x = point[0]
        y = point[1]
        
        # checking for z-value
        if len(point) == 3:
            z = point[2]
        else:
            z = self.odometry.pose.pose.position.z
        
        # calculate errors (x,y,z) between goal position and current drone position
        e_x = x - self.odometry.pose.pose.position.x 
        e_y = y - self.odometry.pose.pose.position.y 
        e_z = z - self.odometry.pose.pose.position.z 
        

        e_x, e_y = self.rotationShift(e_x, e_y, self.odometry.pose.pose.orientation.z)

        cmd_vel = Twist()
        
        # p - controller for position "navigation"
        while abs(e_x) > tol or abs(e_y) > tol or abs(e_z > tol):

            q_x = e_x * p_x
            q_y = e_y * p_y
            q_z = e_z * p_z
            
            cmd_vel.linear.x = q_x
            cmd_vel.linear.y = q_y
            cmd_vel.linear.z = q_z
            
            self.cmd_publisher.publish(cmd_vel)

            e_x = x - self.odometry.pose.pose.position.x # Error in x
            e_y = y - self.odometry.pose.pose.position.y # Error in y
            e_z = z - self.odometry.pose.pose.position.z # Error in z

            e_x, e_y = self.rotationShift(e_x, e_y, self.odometry.pose.pose.orientation.z)

        else:
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
                
            self.cmd_publisher.publish(cmd_vel)

    # controller for planar rotational shift (x-y-plane, rotation about z)
    def rotationShift(self, x, y, theta):
        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) # Drehmatrix
        rot = np.linalg.inv(rot) # Inverse Drehmatrix
        new_x = (rot[0][0] * x) + (rot[0][1] * y)
        new_y = (rot[1][0] * x) + (rot[1][1] * y)
        return new_x, new_y

    # calling for and saving altitude values associated with the current segment
    def sonar_callback(self, data):

        print(round(data.range, 2))
        # current_sm = self.calculate_current_segment()
        # tol = 1 # 1m
        # if abs(self.odometry.pose.pose.position.x - current_sm[0]) < tol and abs(self.odometry.pose.pose.position.y - current_sm[1]) < tol:
        #     # sind im Bereich eines Segmentes -> Wert wird verwendet
        #     index = self.currentYs.index(current_sm[1])
        #     if len(self.heightsForSpalte) < index + 1:
        #         self.heightsForSpalte.append([])
        #     else:
        #         self.heightsForSpalte[index].append(round(data.range, 2))


    def pose_callback(self, data):

        # save actual drone position (x,y,z [m]) and orientation (z [°]) in global odometry variable
        self.odometry.pose.pose.position.x = round(data.pose.pose.position.x, 2)
        self.odometry.pose.pose.position.y = round(data.pose.pose.position.y, 2)
        self.odometry.pose.pose.position.z = round(data.pose.pose.position.z, 2)

        self.odometry.pose.pose.orientation.z = round(self.quaterionToRads(data), 2)

    # transform coordinates and angles from quaternion to values in radiant
    def quaterionToRads(self, data): # aus ui_hector_quad.py
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yawZActual = math.atan2(t3, t4)
        if yawZActual < 0:
            yawZActual = 2*math.pi + yawZActual

        return yawZActual

    # calculate the current segment based on the drone position
    def calculate_current_segment(self):
        current_y = self.odometry.pose.pose.position.y
        current_sm = [self.currentX, min(self.currentYs, key=lambda x:abs(x-current_y))]
        return current_sm



if __name__ == '__main__':

    try:
        HectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")
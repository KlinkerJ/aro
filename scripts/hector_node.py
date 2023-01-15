#!/usr/bin/env python3

import sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import Range, LaserScan
import numpy as np
import math
import db
import sektoren
import time

class HectorNode(object):
    def __init__(self, dronetype):

        self.dronetype = dronetype # 1 = measurement, 2 = fertilization
        self.measurement_active = False
        self.fertilize_gram = 1000
        self.heights = []
        self.cmd_vel = Twist()
        self.range = 0 

        rospy.init_node('hector_node')

        self.odometry = Odometry()
        rospy.Subscriber("/ground_truth/state", Odometry, self.pose_callback)  # topic published with 100 Hz

        rospy.Subscriber("/sonar_height", Range, self.sonar_callback)  # 10 Hz

        # release drone with empty pub on '/release' - topic
        rospy.Subscriber("/release", Empty, self.release_callback)

        # pub drone control velocity
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 

        # pub drone control fertilizer stock quantity [max, actual, last fertilization quantity]
        if dronetype == 2: self.fertilizer_publisher = rospy.Publisher("/fertilizer_stock", Vector3, queue_size=1) 
        
        # get drone home position and save to list
        home_pos = rospy.get_param(f'~uav{dronetype}')
        self.home_pos = list(home_pos['home'].values()) # x,y,z
        rospy.loginfo(f'uav{dronetype} home: {self.home_pos}')
        
        # get nested dictornary for corners from yaml and save to global list
        corners = rospy.get_param('~corners')
        self.corners = [list(corners[key].values()) for key in corners.keys()] # keys: p1-p4; values x,y

        # get height for drone to do their job
        self.working_height = rospy.get_param('~height') # pre-working_height
        self.working_height_set = False

        # get segment_size
        self.segment_size = rospy.get_param('~segment_size')
        
        # get margin (how much southern should the drone start to first segment)
        self.margin = rospy.get_param('~margin')
        
        # get sonar offset (sensor mounted under drone origin)
        self.sonar_offset = rospy.get_param('~sonar_offset')  # m


    def release_callback(self, empty_msg):
        # release drone with empty pub on '/release' - topic

        rospy.logwarn("release of drone: " + str(self.dronetype))
        
        if self.dronetype == 1:

            self.determine_working_height()
            self.calulate_segments()
            self.start_measure()
        
        elif self.dronetype == 2:

            self.fertilize()
        
        self.land(self.home_pos)

        return


    def calulate_segments(self):
        # calculates segments from corners and saves them in db

        if not len(self.corners) == 4:
            return
         
        rospy.logwarn(str(self.corners[0]) + str(self.corners[1]))
        columns = sektoren.getSektorForEckpunkte(
            self.corners[0], self.corners[1], self.corners[2], self.corners[3], self.segment_size) # muss segment size ein Int sein??

        rospy.logwarn(str(columns))
        
        db.create_segments_in_db(columns)
        rospy.loginfo("Segments created in DB.")
        return


    def determine_working_height(self):
        rospy.loginfo('Determining working height.')

        # via sonar
        self.flyToPosition([None, None, self.working_height])
        f_m_x = (self.corners[0][0] - self.corners[3][0]) / 2 + 2 # +2 (offset of first plant model)
        f_m_y = (self.corners[1][1] - self.corners[0][1]) / 2 + 2

        rospy.logwarn('field mid point: ' + str(f_m_x) + str(f_m_y))
        self.flyToPosition([f_m_x, f_m_y])

        while self.range >= 1: # min height above plants in middle of the field
            self.flyToPosition([None, None, self.odometry.pose.pose.position.z - 0.5])

        self.working_height_set = True
        self.working_height = self.odometry.pose.pose.position.z
        return


    def start_measure(self): 
        rospy.loginfo("Starting the measurement.")
        
        # write constants to object
        constants = db.get_constants()
        self.constants = constants

        # get first point
        first_point = db.calculate_first_point(
            constants['min_x'], constants['min_y'], constants['max_y'], self.margin, self.segment_size)

        # fly to first point
        self.flyToPosition(first_point, vmax=2)

        # set constant to enable saving of height in sonar_callback
        self.measurement_active = True

        finished = False
        while not finished:
            # get next point
            if len(self.heights) > 0:
                # save heights in DB
                db.save_heights_after_measurement(self.heights, self.segment_size)
                self.heights = []
                rospy.sleep(2) # let the database write its data to file
            next_point = db.calculate_next_point(
                constants['min_x'], constants['max_x'], 
                constants['min_y'], constants['max_y'], 
                constants['segmentsize'], constants['tolerance'], self.margin, 
                self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y)
            #rospy.logwarn("Nextpoint: " + str(next_point))
            if next_point == []:
                finished = True
                break
            # fly to next point
            self.flyToPosition(next_point, vmax=0.6, ramp=0.25)
        
        self.measurement_active = False

        return


    def fertilize(self):
        rospy.loginfo("Starting Fertilization.")

        fertilizer_msg = Vector3()
        fertilizer_msg.x = 1000 # max value [g]
        fertilizer_msg.y = 1000 # remaining quantity
        fertilizer_msg.z = 0 # fertilization quantity on last segment
        self.fertilizer_publisher.publish(fertilizer_msg)

        constants = db.get_constants()
        self.constants = constants
        fertilize_margin = 0.5
        height_goal = 2
        p_factor = 4
        fertilize_factor = 30 # g per second

        # path generation
        v1, v2, v3 = db.generate_path(self.home_pos)
        v1[0].append(self.home_pos[2] + fertilize_margin)
        v1[-1].append(self.home_pos[2] + fertilize_margin)
        for point in v1:
            try:
                segment_height = db.get_current_segment(point[0], point[1], self.segment_size / 2).height
                if (self.odometry.pose.pose.position.z > segment_height + fertilize_margin):
                    # current position is higher then next segment
                    z = self.odometry.pose.pose.position.z
                    self.flyToPosition([point[0], point[1], z], vmax=0.8, ramp=0.25, tol=0.1) # fly to segment
                    self.flyToPosition([point[0], point[1], segment_height + fertilize_margin], vmax=0.8, ramp=0.25, tol=0.1) # fly to segment height
                else:
                    # current position is lower then next segment
                    x = self.odometry.pose.pose.position.x
                    y = self.odometry.pose.pose.position.y
                    z = segment_height + fertilize_margin
                    self.flyToPosition([x, y, z], vmax=0.8, ramp=0.25, tol=0.05) # fly to segment height
                    self.flyToPosition([point[0], point[1], z], vmax=0.8, ramp=0.25, tol=0.1) # fly to segment
                
                p = abs(segment_height - height_goal) * p_factor
                new_fertilize_gram = self.fertilize_gram - (p * fertilize_factor)
                if new_fertilize_gram < 0:
                    # if less then 0g left after fertilizing this segment, recharge first
                    last_position = [self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, self.odometry.pose.pose.position.z]
                    self.flyToPosition([last_position[0], last_position[1], self.working_height], vmax=0.8, ramp=0.25)
                    self.flyToPosition([self.home_pos[0], self.home_pos[1], self.working_height], ramp=0.25)
                    self.flyToPosition([self.home_pos[0], self.home_pos[1], self.home_pos[2]], ramp=0.25, tol=0.05)
                    rospy.logwarn("Recharching Fertilize Container!")
                    rospy.sleep(10)
                    self.fertilize_gram = 1000
                    fertilizer_msg.y = self.fertilize_gram
                    self.fertilizer_publisher.publish(fertilizer_msg)
                    self.flyToPosition([self.home_pos[0], self.home_pos[1], self.working_height], ramp=0.25)
                    self.flyToPosition([last_position[0], last_position[1], self.working_height], ramp=0.25)
                    self.flyToPosition(last_position, vmax=0.8, ramp=0.25, tol=0.05)
                    
                fertilize_quantity = p * fertilize_factor
                self.fertilize_gram = self.fertilize_gram - fertilize_quantity
                rospy.loginfo("Fertilizing with " + str(fertilize_quantity) + "g. Remaining: " + str(self.fertilize_gram))
                rospy.sleep(p)
                
                fertilizer_msg.y = self.fertilize_gram 
                fertilizer_msg.z = fertilize_quantity 
                self.fertilizer_publisher.publish(fertilizer_msg)

            except Exception as e:
                print(e)
                self.flyToPosition(point, vmax=0.6, ramp=0.25, tol=0.05)
        return


    def flyToPosition(self, point, tol=0.2, p_x=0.2, p_y=0.2, p_z=0.2, vmax=1.0, ramp=0.4):

        # set self.pid_time to allow calculation of cycle time -> needed for velocity limit
        self.pid_time = time.time()

        # get x and y value [m] from goal position
        rospy.logwarn(f'fly to: {point}')

        # variable point format for points in 2D and 3D
        x = self.odometry.pose.pose.position.x if point[0] == None else point[0]
        y = self.odometry.pose.pose.position.y if point[1] == None else point[1]
        z = self.odometry.pose.pose.position.z if len(point) < 3 else point[2]

        # calculate errors (x,y,z) between goal position and current drone position
        e_x = x - self.odometry.pose.pose.position.x
        e_y = y - self.odometry.pose.pose.position.y
        e_z = z - self.odometry.pose.pose.position.z
         
        e_x, e_y = self.rotationShift(
            e_x, e_y, self.odometry.pose.pose.orientation.z)  # rotate error vector to match drone orientation

        cmd_vel = Twist()

        # p - controller for position "navigation"
        while abs(e_x) > tol or abs(e_y) > tol or abs(e_z) > tol:

            q_x = e_x * p_x
            q_y = e_y * p_y
            q_z = e_z * p_z

            # limit velocity, calulate length of vector -> if longer than vmax, scale vector to vmax
            v = math.sqrt(q_x**2 + q_y**2 + q_z**2)
            if v > vmax:
                q_x = q_x * vmax / v
                q_y = q_y * vmax / v
                q_z = q_z * vmax / v
                pass
                    
            # limit acceleration, calculate length of vector -> if larger than 140% of last cycle, scale vector to 140% of last cycle
            try:
                v = math.sqrt(q_x**2 + q_y**2 + q_z**2)
                v_old = math.sqrt(self.cmd_vel.linear.x**2 + self.cmd_vel.linear.y**2 + self.cmd_vel.linear.z**2)
                f = v / v_old
                if f > 1 + ramp:
                    print("Limiting acceleration, factor:", f)
                    q_x = q_x / v * (1 + ramp)
                    q_y = q_y / v * (1 + ramp)
                    q_z = q_z / v * (1 + ramp)
            except Exception as e:
                print(e)

            cmd_vel.linear.x = q_x
            cmd_vel.linear.y = q_y
            cmd_vel.linear.z = q_z

            self.cmd_publisher.publish(cmd_vel)
            self.cmd_vel = cmd_vel

            e_x = x - self.odometry.pose.pose.position.x  # Error in x
            e_y = y - self.odometry.pose.pose.position.y  # Error in y
            e_z = z - self.odometry.pose.pose.position.z  # Error in z

            e_x, e_y = self.rotationShift(
                e_x, e_y, self.odometry.pose.pose.orientation.z)

        else:
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.linear.z = 0

            self.cmd_publisher.publish(cmd_vel)


    def land(self, pos):
        rospy.loginfo(f'landing to: {pos}')
        self.flyToPosition(pos)
        self.flyToPosition([None, None, 0])


    def rotationShift(self, x, y, theta):
        # controller for planar rotational shift (x-y-plane, rotation about z)

        rot = np.array([[np.cos(theta), -np.sin(theta)],
                       [np.sin(theta), np.cos(theta)]])  # Drehmatrix
        rot = np.linalg.inv(rot)  # Inverse Drehmatrix
        new_x = (rot[0][0] * x) + (rot[0][1] * y)
        new_y = (rot[1][0] * x) + (rot[1][1] * y)
        return new_x, new_y


    def sonar_callback(self, data):
        # calling for and saving altitude values associated with the current segment

        if self.dronetype == 1:
            if self.measurement_active:
                self.heights.append([self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, 
                round(self.odometry.pose.pose.position.z - data.range - self.sonar_offset, 2)])
            
            # determine working height via sonar
            if self.measurement_active == False and self.working_height_set == False:
               self.range = data.range


    def pose_callback(self, data):

        # save actual drone position (x,y,z [m]) and orientation (z [°]) in global odometry variable
        self.odometry.pose.pose.position.x = round(
            data.pose.pose.position.x, 2)
        self.odometry.pose.pose.position.y = round(
            data.pose.pose.position.y, 2)
        self.odometry.pose.pose.position.z = round(
            data.pose.pose.position.z, 2)

        self.odometry.pose.pose.orientation.z = round(
            self.quaterionToRads(data), 2)

   
    def quaterionToRads(self, data):
        # transform coordinates and angles from quaternion to values in radiant

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


if __name__ == '__main__':

    filename = sys.argv[0]
    dronetype = sys.argv[1]
    print(type(dronetype))

    try:
        HectorNode(int(dronetype))
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")
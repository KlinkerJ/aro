#!/usr/bin/env python3

import sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import Range
import numpy as np
import math
import db
import sektoren
import time

class HectorNode(object):
    def __init__(self, dronetype):

        self.dronetype = dronetype # 1 = measurement, 2 = fertilization
        self.sonar_offset = 0.17  # m not used
        self.drone_z_offset = 0.28  # m not used
        self.corners = []  # array of corners, empty at start <-- kann aber auch gut über yaml übergeben werden siehe weiter unten
        self.battery_time = time.time()
        self.measurement_active = False
        self.battery = 100
        self.current_segment = 0
        self.heights = []
        self.cmd_vel = Twist()
        self.fertilization_speed = 0


        rospy.init_node('hector_node')

        self.odometry = Odometry()
        rospy.Subscriber("/ground_truth/state", Odometry,
                         self.pose_callback)  # topic published with 100 Hz

        rospy.Subscriber("/sonar_height", Range, self.sonar_callback)  # 10 Hz

        # release drone with empty pub on '/release' - topic
        rospy.Subscriber("/release", Empty, self.release_callback)
        
        # pub drone control velocity
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
        
        # get drone home position and save to list
        home_pos = rospy.get_param(f'~uav{dronetype}')
        self.home_pos = list(home_pos['home'].values()) # x,y
        rospy.logwarn(f'uav{dronetype} home: {self.home_pos}')
        
        # get nested dictornary for corners from yaml and save to global list
        corners = rospy.get_param('~corners')
        self.corners = [list(corners[key].values()) for key in corners.keys()] # keys: p1-p4; values x,y

        # get height for drone to do their job
        self.working_height = rospy.get_param('~height')

        # get segment_size
        self.segment_size = rospy.get_param('~segment_size')
        
        # get marfing (how much southern should the drone start to first segment)
        self.margin = rospy.get_param('~margin')


    def release_callback(self, empty_msg):
        
        # release drone with empty pub on '/release' - topic
        rospy.logwarn("release of drone: " + str(self.dronetype))
        
        # start drone and fly at working height
        self.flyToPosition([None, None, self.working_height])

        # probably not the best technique: we should self-detect the height bei slowly approaching the ground and detecting the height with sonar
        # working height: height at which the sonar detects plants with a margin
        # not yet implemented. if the drone is too high, it should descend slowly until it is at the working height
        
        if self.dronetype == 1:

            self.calulate_segments()
            self.start_measure()
        
        elif self.dronetype == 2:

            # fly to all points and fertilize
            self.fertilize()
        
        self.land(self.home_pos)
        return

    # calculates segments from corners and saves them in db
    def calulate_segments(self):
        if not len(self.corners) == 4:
            return
         
        rospy.logwarn(str(self.corners[0]) + str(self.corners[1]))
        columns = sektoren.getSektorForEckpunkte(
            self.corners[0], self.corners[1], self.corners[2], self.corners[3], self.segment_size) # muss segment size ein Int sein??

        rospy.logwarn(str(columns))
        
        db.create_segments_in_db(columns)
        rospy.loginfo("Segments created in DB.")
        return


    def start_measure(self): 
        rospy.loginfo("Starting the measurement.")
        constants = db.get_constants()
        # write constants to object
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
            next_point = db.calculate_next_point(
                constants['min_x'], constants['max_x'], 
                constants['min_y'], constants['max_y'], 
                constants['segmentsize'], constants['tolerance'], self.margin, 
                self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y)
            rospy.logwarn("Nextpoint: " + str(next_point))
            if next_point == []:
                finished = True
                break
            # fly to next point
            self.flyToPosition(next_point, vmax=0.6, ramp=0.25)

        return

    def fertilize(self):
        rospy.loginfo("Starting Fertilization.")
        constants = db.get_constants()
        # write constants to object
        self.constants = constants

        # test path generation
        v1, v2, v3 = db.generate_path()
        rospy.loginfo(str(v1))
        for point in v1:
            rospy.loginfo("Using V1 for Path Generation: " + str(point))
            # using v1 path generation
            self.flyToPosition(point, vmax=0.6, ramp=0.25)
        return

    def flyToPosition(self, point, tol=0.2, p_x=0.2, p_y=0.2, p_z=0.2, vmax=1.0, ramp = 0.4):

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

            #rospy.loginfo('flying')

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
            
            if self.dronetype == 2 and self.fertilization_speed > 0:
                rospy.logwarn("Limiting Fertiliazion Speed")
                v = math.sqrt(q_x**2 + q_y**2 + q_z**2)
                if v > self.fertilization_speed:
                    q_x = q_x * self.fertilization_speed / v
                    q_y = q_y * self.fertilization_speed / v
                    q_z = q_z * self.fertilization_speed / v
                    
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
        self.flyToPosition([None, None, 0]) # 1D list --> fly in z-axis


    # controller for planar rotational shift (x-y-plane, rotation about z)
    def rotationShift(self, x, y, theta):
        rot = np.array([[np.cos(theta), -np.sin(theta)],
                       [np.sin(theta), np.cos(theta)]])  # Drehmatrix
        rot = np.linalg.inv(rot)  # Inverse Drehmatrix
        new_x = (rot[0][0] * x) + (rot[0][1] * y)
        new_y = (rot[1][0] * x) + (rot[1][1] * y)
        return new_x, new_y

    # calling for and saving altitude values associated with the current segment
    def sonar_callback(self, data):
        # print("Sonar Height:", round(data.range, 2))
        if self.measurement_active and self.dronetype == 1:
            try:
                current_segment = db.get_current_segment(
                    self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, self.constants['tolerance'])  # get current segment - not sure if fast enough via DB
            except:
                #print("No current Segment")
                if len(self.heights) > 0:
                    last_segment = db.get_segment_for_id(self.current_segment)
                    db.save_heights_for_segment(last_segment, self.heights)
                    self.current_segment = 0
                    self.heights = []
            else:
                if self.current_segment == 0:
                    # set new segment
                    self.current_segment = current_segment.id
                    self.heights = []
                    self.heights.append(round(self.odometry.pose.pose.position.z - data.range, 2))
                elif self.current_segment == current_segment.id:
                    # append height
                    self.heights.append(round(self.odometry.pose.pose.position.z - data.range, 2))
                else:
                    # save heights for last segment
                    last_segment = db.get_segment_for_id(self.current_segment)
                    db.save_heights_for_segment(last_segment, self.heights)
                    # set new segment
                    self.current_segment = current_segment.id
                    self.heights = []
                    self.heights.append(round(self.odometry.pose.pose.position.z - data.range, 2))

    def pose_callback(self, data):
        # battery calculation via time (0.001% per second)
        if not self.battery_time:
            self.battery_time = time.time()
        else:
            self.battery -= (time.time() - self.battery_time) * 0.00001
            self.battery_time = time.time()

        # battery calculation via flown distance
        diff_x = abs(self.odometry.pose.pose.position.x -
                     data.pose.pose.position.x)
        diff_y = abs(self.odometry.pose.pose.position.y -
                     data.pose.pose.position.y)
        distance = math.sqrt(diff_x**2 + diff_y**2)  # flown distance
        # 0.0001 is the battery consumption per meter (0.01% per meter)
        self.battery -= distance * 0.0001

        # we should include something to charge the battery at position x,y,z as its homebase

        # save actual drone position (x,y,z [m]) and orientation (z [°]) in global odometry variable
        self.odometry.pose.pose.position.x = round(
            data.pose.pose.position.x, 2)
        self.odometry.pose.pose.position.y = round(
            data.pose.pose.position.y, 2)
        self.odometry.pose.pose.position.z = round(
            data.pose.pose.position.z, 2)

        self.odometry.pose.pose.orientation.z = round(
            self.quaterionToRads(data), 2)
        

        #Ursache für das nicht anhalten der Drohne 2!!!
        # if self.dronetype == 2:
        #     try:
        #         current_segment = db.get_current_segment(
        #             self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, self.constants['tolerance'])  # get current segment - not sure if fast enough via DB
        #         self.fertilization_speed = (3 - current_segment.height) * 10
        #         rospy.loginfo("Fertilization Speed" + str(self.fertilization_speed))
        #     except:
        #         self.fertilization_speed = 0

    # transform coordinates and angles from quaternion to values in radiant
    def quaterionToRads(self, data):  # aus ui_hector_quad.py
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

#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import Range
import numpy as np
import math
import db
import sektoren

# vllt argument für remote controll?!


class HectorNode(object):
    def __init__(self):

        self.sonar_offset = 0.17  # m
        self.drone_z_offset = 0.28  # m

        rospy.init_node('hector_node')

        self.odometry = Odometry()
        rospy.Subscriber("/ground_truth/state", Odometry,
                         self.pose_callback)  # topic published with 100 Hz

        rospy.Subscriber("/sonar_height", Range, self.sonar_callback)  # 10 Hz

        rospy.Subscriber("/release", Empty, self.release_callback)

        #rospy.Subscriber("/button_pressed", Empty, self.button_pressed_callback)

        #rospy.Subscriber("/measure", Empty, self.start_measure_callback)

        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist)
        # self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

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
        self.flypoints = [[0, 0, 10], [20, 0], [20, 20], [0, 20]]

        # flypoints müssen aus den segmentmittelpunkten erstellt werden

    def release_callback(self, empty_msg):

        # release drone with empty pub on '/release' - topic
        for point in self.flypoints:
            self.flyToPosition(point)

        self.button_pressed()

        return

    def button_pressed(self): # _callback
        # initialize array if not existing
        if self.corners is not []:
            self.corners = []
        # add corner to array
        corner = [self.odometry.pose.pose.position.x,
                  self.odometry.pose.pose.position.y]
        self.corners.append(corner)

        self.calulate_segments()

        return

    def calulate_segments(self): # _callback
        if not self.corners.length == 4:
            return
        segmentsize = 2
        columns = sektoren.getSektorForEckpunkte(
            self.corners[0], self.corners[1], self.corners[2], self.corners[3], segmentsize)
        db.create_segments_in_db(columns)
        print("Segments created in DB")

        self.start_measure()

        return

    def start_measure(self): # _callback
        constants = db.get_constants()
        # write constants to object
        self.constants = constants
        # get first point
        margin = 2  # 2m margin, how much southern should the drone start to first segment
        first_point = db.calculate_first_point(
            constants.min_x, constants.min_y, constants.max_y, margin)

        finished = False
        while not finished:
            # get next point
            next_point = db.calculate_next_point(
                constants.min_x, constants.max_x, constants.min_y, constants.max_y, constants.segmentsize, constants.tolerance, margin, self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y)
            if not next_point:
                finished = True
            # fly to next point
            # simulate flight
            print("Flying to next point:", next_point)
            self.flyToPosition(next_point)
        return

    def flyToPosition(self, point, tol=0.2, p_x=0.2, p_y=0.2, p_z=0.2):

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

        e_x, e_y = self.rotationShift(
            e_x, e_y, self.odometry.pose.pose.orientation.z)

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

            e_x = x - self.odometry.pose.pose.position.x  # Error in x
            e_y = y - self.odometry.pose.pose.position.y  # Error in y
            e_z = z - self.odometry.pose.pose.position.z  # Error in z

            e_x, e_y = self.rotationShift(
                e_x, e_y, self.odometry.pose.pose.orientation.z)

        else:
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0

            self.cmd_publisher.publish(cmd_vel)

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
        print("Sonar Height:", round(data.range, 2))
        # current_segment = db.get_current_segment(
        #     self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, self.constants.tolerance)
        # if self.current_segment == current_segment.id:
        #     # append height
        #     self.heights.append(round(data.range, 2))
        # else:
        #     # save heights for last segment
        #     last_segment = db.get_segment_by_id(self.current_segment)
        #     db.save_heights(self.heights, last_segment)
        #     # set new segment
        #     self.current_segment = current_segment.id
        #     self.heights = []
        #     self.heights.append(round(data.range, 2))

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

    try:
        HectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")

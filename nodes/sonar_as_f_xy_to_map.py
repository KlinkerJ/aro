#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Status
from geometry_msgs.msg import Twist

class SonarToMap(object):
    def __init__(self):

        rospy.init_node('sonar_to_map')
        
        # load params from status_to_hector.yaml file
        self.inputs = rospy.get_param('~inputs')
        self.scales = rospy.get_param('~scales')

        self.axis = rospy.get_param('~axis')
        self.axis_attrs = list(self.axis.values())

        self.button = rospy.get_param('~button')
        self.button_attrs = list(self.button.values())

        # /status
        self.prev_status = Status()
        rospy.Subscriber('status', Status, self.cb_status, queue_size=1)

        # /cmd_vel
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)


    def cb_status(self, msg):

        self.cb_axis(msg)

        self.cb_button(msg)
    

    def cb_axis(self, msg):
        
        # get input values for each axis attribute
        input_vals = {}
        for attr in self.axis_attrs:
            input_vals[attr] = getattr(msg, attr)

        cmd_vel_rc = Twist()

        for vel_type in self.inputs:
            vel_vec = getattr(cmd_vel_rc, vel_type)
            for k, expr in self.inputs[vel_type].items():
                scale = self.scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                setattr(vel_vec, k, scale * val)

        self.pub_vel.publish(cmd_vel_rc)


    def cb_button(self, msg):

        # detect change in button status
        for attr in self.button_attrs:
            if getattr(msg, attr) is not getattr(self.prev_status, attr): # flag in button status detected
                if getattr(msg, attr) == True: # positive flag in one button status detected

                    # buttons without service call
                    if attr == 'button_trackpad':
                        rospy.loginfo("Battery: %s %%", ( msg.battery_percentage * 100))
                        rospy.loginfo("USB: %s",  msg.plug_usb)
                        break
                    
        self.prev_status =  msg



if __name__ == '__main__':

    try:
        SonarToMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")
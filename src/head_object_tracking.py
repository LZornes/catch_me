#!/usr/bin/env python
from collections import deque

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('geometry_msgs')

import rospy
from control_msgs.msg import PointHeadAction
from control_msgs.msg import PointHeadGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from ar_track_alvar.msg import AlvarMarker

class HeadObjectTracking():

    def __init__(self):
        name_space = '/head_traj_controller/point_head_action'
        self.head_client = SimpleActionClient(name_space, PointHeadAction)
        self.head_client.wait_for_server()

        self.curr_tracking_point = Point(1, 0, 0.5)
        self.point_head(self.curr_tracking_point.x, self.curr_tracking_point.y, self.curr_tracking_point.z)
        rospy.Subscriber('catch_me_destination_publisher', AlvarMarker, self.new_tracking_data)

    def new_tracking_data(self, marker):
        """
        Adds a new tracking data point for the head.

        Points the head to a point taken as a moving average over some number of
        previous tracking data points.
        """
        pos = marker.pose.pose.position

        OLD_DATA_WEIGHT = .3

        # calculate the moving average of the x, y, z positions
        tracking_point = self.curr_tracking_point
        avg_x = (self.curr_tracking_point.x * OLD_DATA_WEIGHT) + (pos.x * (1 - OLD_DATA_WEIGHT))
        avg_y = (self.curr_tracking_point.y * OLD_DATA_WEIGHT) + (pos.y * (1 - OLD_DATA_WEIGHT))
        avg_z = (self.curr_tracking_point.z * OLD_DATA_WEIGHT) + (pos.z * (1 - OLD_DATA_WEIGHT))

        # make a new averaged point to track and point the head there
        self.curr_tracking_point = Point(avg_x, avg_y, avg_z)
        self.point_head(avg_x, avg_y, avg_z)

    def point_head(self, x, y, z):
        """
        Point the head to the specified point
        """
        head_goal = PointHeadGoal()
        head_goal.target.header.frame_id = '/torso_lift_link'
        head_goal.max_velocity = .3
        # The transform seems to aim high. Move it down a little...
        head_goal.target.point = Point(x, y, z - .4)

        rospy.logdebug('Moving head to\n' + str(head_goal.target.point))
        
        self.head_client.send_goal(head_goal)
#        if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
#            rospy.logwarn('Head action unsuccessful.')

if __name__=='__main__':
    rospy.init_node('catch_me_head_tracking_node')
    head_tracking = HeadObjectTracking()
    rospy.spin()

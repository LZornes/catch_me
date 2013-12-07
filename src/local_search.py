#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('geometry_msgs')

import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import Point
from ar_track_alvar.msg import AlvarMarker
from catch_me.msg import *

import math

#TODO: import point stamped

class LocalSearch():
  VISUAL_FIELD_SIZE = 40
  MIN_HEAD_ANGLE = -140
  MAX_HEAD_ANGLE = 140

  _feedback = LocalSearchFeedback()
  _result = LocalSearchResult()

  def __init__(self):
    self._action_name = 'local_search'
    self.found_marker = False
    self.tracking_started = False
    
    #initialize head mover
    name_space = '/head_traj_controller/point_head_action'
    self.head_client = SimpleActionClient(name_space, PointHeadAction)
    self.head_client.wait_for_server()
    rospy.loginfo('%s: Action client for PointHeadAction running' % self._action_name)

    #initialize tracker mark
    rospy.Subscriber('catch_me_destination_publisher', AlvarMarker, self.marker_tracker)
    rospy.loginfo('%s: subscribed to AlvarMarkers' % self._action_name)
    
    #initialize this client
    self._as = SimpleActionServer(self._action_name, LocalSearchAction, execute_cb=self.run, auto_start=False)
    self._as.start()
    rospy.loginfo('%s: started' % self._action_name)
    
  def marker_tracker(self, marker):
    if self.tracking_started:
      self.found_marker = True  
      rospy.loginfo('%s: marker found' % self._action_name)
    
  def run(self, goal):
    success = False
    self.tracking_started = True
    self.found_marker = False

    rospy.loginfo('%s: Executing search for AR marker' % self._action_name)

    # define a set of ranges to search
    search_ranges = [
      # first search in front of the robot
      (0, self.VISUAL_FIELD_SIZE),
      (self.VISUAL_FIELD_SIZE, -self.VISUAL_FIELD_SIZE),
      # then search all directions
      (-self.VISUAL_FIELD_SIZE, self.MAX_HEAD_ANGLE),
      (self.MAX_HEAD_ANGLE, self.MIN_HEAD_ANGLE),
      (self.MIN_HEAD_ANGLE, 0)
    ]
    range_index = 0

    #success = self.search_range(*(search_ranges[range_index]))
    
    while (not success) and range_index < len(search_ranges) - 1:
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Premepted' % self._action_name)
        self._as.set_preempted()
        break
      range_index = range_index + 1
      success = self.search_range(*(search_ranges[range_index]))
    

    if success:
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded()
    else:
      self._as.set_aborted()
    self.tracking_started = False
    
  def search_range(self, start_angle, end_angle):
    rospy.loginfo("{}: searching range {} {}".format(self._action_name, start_angle, end_angle))
    angle_tick = self.VISUAL_FIELD_SIZE if (start_angle < end_angle) else -self.VISUAL_FIELD_SIZE
    for cur_angle in xrange(start_angle, end_angle, angle_tick):
      if self._as.is_preempt_requested():
	return False
      
      head_goal = self.lookat_goal(cur_angle)
      rospy.loginfo('%s: Head move goal for %s: %s produced' % (self._action_name, str(cur_angle), str(head_goal)))
      self.head_client.send_goal(head_goal)
      self.head_client.wait_for_result(rospy.Duration.from_sec(5.0))
      if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
        rospy.logwarn('Head could not move to specified location')
        break

      # pause at each tick
      rospy.sleep(0.3)
      if (self.found_marker):
        # found a marker!
        return True

    # no marker found
    return False

  def lookat_goal(self, angle):
    head_goal = PointHeadGoal()
    head_goal.target.header.frame_id = '/torso_lift_link'
    head_goal.max_velocity = 0.8

    angle_in_radians = math.radians(angle)
    x = math.cos(angle_in_radians) * 5
    y = math.sin(angle_in_radians) * 5
    z = -0.5
    
    head_goal.target.point = Point(x, y, z)

    return head_goal

if __name__=='__main__':
  rospy.init_node('catch_me_local_search')
  local_search = LocalSearch() 
  rospy.spin()

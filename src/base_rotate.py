#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import Twist, Vector3
from catch_me.msg import *

import math

class BaseRotate():
  def __init__(self):
    self._action_name = 'base_rotate'
    
    #initialize base controller
    topic_name = '/base_controller/command'
    self._base_publisher = rospy.Publisher(topic_name, Twist)

    #initialize this client
    self._as = SimpleActionServer(self._action_name, BaseRotateAction, execute_cb=self.run, auto_start=False)
    self._as.start()
    rospy.loginfo('%s: started' % self._action_name)
    
  def run(self, goal):
    rospy.loginfo('Rotating base')
    count = 0
    r = rospy.Rate(10)
    while count < 70:
      if self._as.is_preempt_requested():
        self._as.set_preempted()
        return

      twist_msg = Twist()
      twist_msg.linear = Vector3(0.0, 0.0, 0.0)
      twist_msg.angular = Vector3(0.0, 0.0, 1.0)

      self._base_publisher.publish(twist_msg)
      count = count + 1
      r.sleep()

    self._as.set_succeeded()    

if __name__=='__main__':
  rospy.init_node('catch_me_base_rotate')
  base_rotate = BaseRotate()
  rospy.spin()

#!/usr/bin/env python

import roslib
roslib.load_manifest('sound_play')
roslib.load_manifest('trajectory_msgs')
roslib.load_manifest('control_msgs')

import rospy
import tf
from actionlib import SimpleActionClient, SimpleActionServer
from geometry_msgs.msg import Twist, Vector3
from catch_me.msg import *
from ar_track_alvar.msg import AlvarMarker
from sound_play.libsoundplay import SoundClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction

import math

class Kill():
  REFRESH_RATE = 20

  def __init__(self):
    self.r1 = [-1.2923559122018107, -0.24199198117104131, -1.6400091364915879, -1.5193418228083817, 182.36402145110227, -0.18075144121090148, -5.948327320167482]
    self.r2 = [-0.6795931033163289, -0.22651111024292614, -1.748569353944001, -0.7718906399352281, 182.36402145110227, -0.18075144121090148, -5.948327320167482]
    self.r3 = [-0.2760036900225221, -0.12322070913238689, -1.5566246267792472, -0.7055856541215724, 182.30397617484758, -1.1580488044879909, -6.249409047256675]

    self.l1 = [1.5992829923087575, -0.10337038946934723, 1.5147248511783875, -1.554810647097346, 6.257580605941875, -0.13119498120772766, -107.10011839122919]
    self.l2 = [0.8243548398730115, -0.10751554070146568, 1.53941949443935, -0.7765233026995009, 6.257580605941875, -0.13119498120772766, -107.10011839122919]
    self.l3 = [0.31464495636226303, -0.06335699084094017, 1.2294536150663598, -0.7714563278010775, 6.730191306327954, -1.1396012223560352, -107.19066045395644]

    self.v = [0] * len(self.r1)

    self.r_joint_names = ['r_shoulder_pan_joint',
                          'r_shoulder_lift_joint',
                          'r_upper_arm_roll_joint',
                          'r_elbow_flex_joint',
                          'r_forearm_roll_joint',
                          'r_wrist_flex_joint',
                          'r_wrist_roll_joint']
    self.l_joint_names = ['l_shoulder_pan_joint',
                          'l_shoulder_lift_joint',
                          'l_upper_arm_roll_joint',
                          'l_elbow_flex_joint',
                          'l_forearm_roll_joint',
                          'l_wrist_flex_joint',
                          'l_wrist_roll_joint']

    self._action_name = 'kill'
    self._tf = tf.TransformListener()
    
    #initialize base controller
    topic_name = '/base_controller/command'
    self._base_publisher = rospy.Publisher(topic_name, Twist)

    #Initialize the sound controller
    self._sound_client = SoundClient()

    # Create a trajectory action client
    r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
    self.r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
    rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
    self.r_traj_action_client.wait_for_server()

    l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
    self.l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
    rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
    self.l_traj_action_client.wait_for_server()

    self.pose = None
    self._marker_sub = rospy.Subscriber('catch_me_destination_publisher', AlvarMarker, self.marker_callback)

    #initialize this client
    self._as = SimpleActionServer(self._action_name, KillAction, execute_cb=self.run, auto_start=False)
    self._as.start()
    rospy.loginfo('%s: started' % self._action_name)

  def marker_callback(self, marker):
    if (self.pose is not None):
      self.pose = marker.pose

  def run(self, goal):
    rospy.loginfo('Begin kill')
    self.pose = goal.pose
    #pose = self._tf.transformPose('/base_link', goal.pose)
    self._sound_client.say('Halt!')

    # turn to face the marker before opening arms (code repeated later)
    r = rospy.Rate(self.REFRESH_RATE)
    while(True):
      pose = self.pose
      if abs(pose.pose.position.y) > .1:
        num_move_y = int((abs(pose.pose.position.y) - 0.1) * self.REFRESH_RATE / .33) + 1
        #print str(pose.pose.position.x) + ', ' + str(num_move_x)
        twist_msg = Twist()
        twist_msg.linear = Vector3(0.0, 0.0, 0.0)
        twist_msg.angular = Vector3(0.0, 0.0, pose.pose.position.y / ( 3 * abs(pose.pose.position.y)))
        for i in range(num_move_y):
          if pose != self.pose:
            break
          self._base_publisher.publish(twist_msg)
          r.sleep()
        pose.pose.position.y = 0
      else:
        break

    # open arms
    traj_goal_r = JointTrajectoryGoal()
    traj_goal_r.trajectory.joint_names = self.r_joint_names
    traj_goal_l = JointTrajectoryGoal()
    traj_goal_l.trajectory.joint_names = self.l_joint_names
    traj_goal_r.trajectory.points.append(JointTrajectoryPoint(positions=self.r1, velocities = self.v, time_from_start = rospy.Duration(2)))
    self.r_traj_action_client.send_goal_and_wait(traj_goal_r, rospy.Duration(3))

    traj_goal_l.trajectory.points.append(JointTrajectoryPoint(positions=self.l1, velocities = self.v, time_from_start = rospy.Duration(2)))
    self.l_traj_action_client.send_goal_and_wait(traj_goal_l, rospy.Duration(3))

    traj_goal_r.trajectory.points[0].positions = self.r2
    self.r_traj_action_client.send_goal(traj_goal_r)

    traj_goal_l.trajectory.points[0].positions = self.l2
    self.l_traj_action_client.send_goal(traj_goal_l)

    self._sound_client.say('You have the right to remain silent.')

    # Keep a local copy because it will update
    #pose = self.pose
    #num_move_x = int((pose.pose.position.x - 0.3) * 10 / .1) + 1
    #print str(pose.pose.position.x) + ', ' + str(num_move_x)
    #twist_msg = Twist()
    #twist_msg.linear = Vector3(.1, 0.0, 0.0)
    #twist_msg.angular = Vector3(0.0, 0.0, 0.0)
    #for i in range(num_move_x):
    #  self._base_publisher.publish(twist_msg)
    #  r.sleep()

    # track the marker as much as we can
    while True:
      pose = self.pose
      # too far away
      if abs(pose.pose.position.x > 1.5):
        rospy.loginfo('Target out of range: ' + str(pose.pose.position.x))
        self._as.set_aborted()
        return;
      # too far to the side -> rotate
      elif abs(pose.pose.position.y) > .1:
        num_move_y = int((abs(pose.pose.position.y) - 0.1) * self.REFRESH_RATE / .33) + 1
        #print str(pose.pose.position.x) + ', ' + str(num_move_x)
        twist_msg = Twist()
        twist_msg.linear = Vector3(0.0, 0.0, 0.0)
        twist_msg.angular = Vector3(0.0, 0.0, pose.pose.position.y / (3 * abs(pose.pose.position.y)))
        for i in range(num_move_y):
          if pose != self.pose:
            break
          self._base_publisher.publish(twist_msg)
          r.sleep()
        pose.pose.position.y = 0
        #twist_msg = Twist()
        #twist_msg.linear = Vector3(0.0, 0.0, 0.0)
        #twist_msg.angular = Vector3(0.0, 0.0, pose.pose.position.y / (5 * abs(pose.pose.position.y)))
        #self._base_publisher.publish(twist_msg)

      # now we are going to move in for the kill, but only until .5 meters away (don't want to run suspect over)
      elif pose.pose.position.x > .5:
        num_move_x = int((pose.pose.position.x - 0.3) * self.REFRESH_RATE / .1) + 1
        #print str(pose.pose.position.x) + ', ' + str(num_move_x)
        twist_msg = Twist()
        twist_msg.linear = Vector3(.1, 0.0, 0.0)
        twist_msg.angular = Vector3(0.0, 0.0, 0.0)
        for i in range(num_move_x):
          if pose != self.pose:
            break
          self._base_publisher.publish(twist_msg)
          r.sleep()
        pose.pose.position.x = 0
        #twist_msg = Twist()
        #twist_msg.linear = Vector3(.1, 0.0, 0.0)
        #twist_msg.angular = Vector3(0.0, 0.0, 0.0)
        #self._base_publisher.publish(twist_msg)
     
      # susupect is within hugging range!!!
      else:
        break
      r.sleep()

    # after exiting the loop, we should be ready to capture -> send close arms goal
    self._sound_client.say("Anything you say do can and will be used against you in a court of law.")

    self.l_traj_action_client.wait_for_result(rospy.Duration(3))
    self.r_traj_action_client.wait_for_result(rospy.Duration(3))

    traj_goal_r.trajectory.points[0].positions = self.r3
    self.r_traj_action_client.send_goal(traj_goal_r)

    traj_goal_l.trajectory.points[0].positions = self.l3
    self.l_traj_action_client.send_goal(traj_goal_l)

    self.l_traj_action_client.wait_for_result(rospy.Duration(3))
    self.r_traj_action_client.wait_for_result(rospy.Duration(3))

    rospy.loginfo('End kill')
    rospy.sleep(20)
    self._as.set_succeeded()

if __name__=='__main__':
  rospy.init_node('catch_me_kill')
  kill = Kill()
  rospy.spin()

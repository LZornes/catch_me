#!/usr/bin/env python

import roslib
roslib.load_manifest('simple_navigation_goals')
roslib.load_manifest('pr2_tuckarm')

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from catch_me import srv
import threading
from catch_me.msg import *
from actionlib_msgs.msg import GoalStatus
import copy
from pr2_common_action_msgs.msg import TuckArmsAction, TuckArmsGoal
from std_srvs.srv import Empty

class CatchMe:
  FOLLOW_DIST = 0.5
  POSITION_THRESHOLD_DIST = 0.1
  KILL_DIST_THRESHOLD = 1

  def __init__(self):
    self._last_move = 0
    self._last_trans_pose = None

    self.tf = tf.TransformListener()

    self.tuck_arm_client = actionlib.SimpleActionClient('catch_me_tuck_arms', TuckArmsAction)
    rospy.loginfo('Waiting for tuck arms action server to start')
    self.tuck_arm_client.wait_for_server()
    rospy.loginfo('Connected to tuck arms action server')

    goal = TuckArmsGoal()
    goal.tuck_left = True
    goal.tuck_right = True
    self.tuck_arm_client.send_goal(goal)

    rospy.loginfo('Waiting for catch_me_destination_service')
    rospy.wait_for_service('catch_me_destination_service')
    rospy.loginfo('Connected to catch_me_destination_service')
#    self.destination_service = rospy.ServiceProxy('catch_me_destination_service', srv.DestinationService)

    rospy.loginfo('Waiting for move_base action to come up')
    self.base_client = actionlib.SimpleActionClient('move_base_local', MoveBaseAction)
    #self.base_client.wait_for_server()
    rospy.loginfo('Connected to move_base action server')

   
    rospy.loginfo('Starting head-searching action')
    self.head_s_client = actionlib.SimpleActionClient('local_search', LocalSearchAction)
    #self.head_s_client.wait_for_server()
    rospy.loginfo('Head-search action client started')

    rospy.loginfo('Starting base_rotate action')
    self.base_r_client = actionlib.SimpleActionClient('base_rotate', BaseRotateAction)
    #self.base_r_client.wait_for_server()
    rospy.loginfo('Base_rotate action client started')

    rospy.loginfo('Starting kill action')
    self.kill_client = actionlib.SimpleActionClient('kill', KillAction)
    self.kill_client.wait_for_server()
    rospy.loginfo('Kill action client started')


    # Don't start until arms are tucked!
    self.tuck_arm_client.wait_for_result(rospy.Duration(30.0))
    threading.Timer(0, self.follow_marker).start()

  def follow_marker(self):
    destination_service = rospy.ServiceProxy('catch_me_destination_service', srv.DestinationService)
    poses = destination_service()
    trans_pose = poses.trans_pose
    pose = poses.pose
    print trans_pose

    # If we can't find a path, spin to clear obstacles, else stop spinning
    #if self.base_client.get_state() == GoalStatus.PENDING:
    #  goal = BaseRotateGoal()
    #  self.base_r_client.send_goal(goal)
    #else:
    #  self.base_r_client.cancel_goal()
    
    distance_to_marker = ((pose.pose.position.x ** 2) + (pose.pose.position.y ** 2)) ** .5
    if (trans_pose is None or
          trans_pose.header.stamp.secs == 0 or
          (trans_pose == self._last_trans_pose and (distance_to_marker >= self.KILL_DIST_THRESHOLD or pose.header.stamp + rospy.Duration(1.5) > rospy.Time.now()))):
      # Either, no marker known, or marker is old
      self._last_move = self._last_move + 1
      if trans_pose == self._last_trans_pose:
        rospy.loginfo('Marker position unchanged')
      else:
        rospy.loginfo('No marker position found')

      if self._last_move > 4 and self.head_s_client.get_state() != GoalStatus.ACTIVE:
        goal = LocalSearchGoal(True)
        self.head_s_client.send_goal(goal)
        self._last_move = 0

    else:
      # New marker seen, try to move to it
      pos = trans_pose.pose.position
      orient = trans_pose.pose.orientation
      pos_frame_id = trans_pose.header.frame_id
      rospy.logdebug(str(pos_frame_id) + '\n' + str(pos.x) + ',' + str(pos.y) + ',' + str(pos.z))

      # obtain rosie's transformed pose and check whether she is close
      # enought to initiate kill sequence
      #robot_pose = self.tf.transformPose('/map', trans_pose)
      #robot_position = robot_pose.pose.pose.position
      #distance_to_marker = self.distance_between(pos, robot_position)
      #if (self._last_trans_pose is not None and
      #      distance_to_marker < self.KILL_DIST_THRESHOLD):
      #  rospy.loginfo('initiating kill sequence')
      #  ''' this goes in some 
      #      - orient body (and head) toward marker
      #      - say 'Halt!'
      #      - open arms
      #      - manually move forward distance_to_marker meters
      #      - close arms
      #      - read miranda rights
      #  '''
      #  rospy.loginfo('finished kill sequence')

      trans_pose_copy = copy.deepcopy(trans_pose)
      #print trans_pose.header.stamp
      #trans_pose.header.stamp = self.tf.getLatestCommonTime(trans_pose.header.frame_id, '/base_link')
      #print trans_pose.header.stamp
      #trans_pose.header.stamp = rospy.Time.now()
      # Hunter: seems to be working, we can play around with value to find an optimal
      #print trans_pose.header.frame_id
      #self.tf.waitForTransform('/base_link', trans_pose.header.frame_id, trans_pose.header.stamp, rospy.Duration(4.0))
      #robot_pose = self.tf.transformPose('/base_link', trans_pose)
      #print str(trans_pose_copy.header.stamp) + ' ' + str(rospy.Time.now())
      if self._last_trans_pose is not None and pose.header.stamp + rospy.Duration(1.5) > rospy.Time.now() and distance_to_marker < self.KILL_DIST_THRESHOLD:
        print distance_to_marker
        rospy.loginfo('Going in for the kill!')
        self._attempt_kill(pose)
      elif (self._last_trans_pose is not None and
            self.distance_between(pos, self._last_trans_pose.pose.position) < self.POSITION_THRESHOLD_DIST):
        rospy.loginfo('Marker found close to last goal sent, NOT sending new goal')
        if self.head_s_client.get_state() != GoalStatus.ACTIVE:
          goal = LocalSearchGoal(True)
          self.head_s_client.send_goal(goal)
      elif trans_pose != self._last_trans_pose or self.base_client.get_state() != GoalStatus.SUCCEEDED:
        rospy.loginfo('Marker position updated.  Moving towards marker.')
        self._last_trans_pose = trans_pose_copy

        self._move_to_marker(trans_pose)


    goal_status = self.base_client.get_state()
    if goal_status == GoalStatus.PENDING or goal_status == GoalStatus.ACTIVE:
      # if we are currently moving/planning, slow update rate
      update_rate = 1
    else:
      # if we are currently a sitting duck, fast update rate
      update_rate = 0.5
    threading.Timer(update_rate, self.follow_marker).start()

  def _attempt_kill(self, pose):
    self.base_client.cancel_all_goals()
    goal = KillGoal()
    goal.pose = pose # Pose may be out of date, not much we can do since tf keeps failing...
    self.kill_client.send_goal_and_wait(goal)
    goal = TuckArmsGoal()
    goal.tuck_left = True
    goal.tuck_right = True
    self.tuck_arm_client.send_goal(goal)
    self.tuck_arm_client.wait_for_result(rospy.Duration(30.0))
    # Clear the arms from the cost map
    clear_service = rospy.ServiceProxy('/move_base_local_node/clear_costmaps', Empty)
    clear_service()


  def _move_to_marker(self, trans_pose):
    pose = trans_pose

    goal = MoveBaseGoal()
    goal.target_pose = pose

    # The robot can't move in these directions. Clear them so it's not confused
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0


    # If the marker position is fresh, stay behind it so we don't collide.
    # Use trans_pose to get original time
    if self._last_trans_pose.header.stamp + rospy.Duration(2) > rospy.Time.now():
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - self.FOLLOW_DIST


    rospy.logdebug('Sending new goal. x: ' + str(goal.target_pose.pose.position.x) + ', y: ' + str(goal.target_pose.pose.position.y))
    # Cancel old goal if it exists.
    self.base_client.cancel_all_goals()
    self.base_client.send_goal(goal)
    #self.base_client.wait_for_result()
    #rospy.loginfo(str(self.base_client.get_result()))

  def distance_between(self, position1, position2):
    return ((position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2) ** .5

if __name__=='__main__':
    rospy.init_node('catch_me_node')
    dest_pub = CatchMe()
    rospy.spin()
 

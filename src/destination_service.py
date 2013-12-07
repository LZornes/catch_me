#!/usr/bin/env python

import rospy
import tf
from ar_track_alvar.msg import AlvarMarker
from catch_me import srv

class DestinationService:
  def __init__(self):
    self.tf = tf.TransformListener()
    # Let the transform build...
    rospy.sleep(10)
    # I need a better default value...
    self.pose = None
    self.trans_pose = None
    rospy.Subscriber('catch_me_destination_publisher', AlvarMarker, self.marker_cb)
    dest_serv = rospy.Service('catch_me_destination_service', srv.DestinationService, self.service_cb)

  def marker_cb(self, marker):
    rospy.logdebug('AR Marker Pose updating')    
    self.pose = marker.pose
    self.pose.header = marker.header # Marker has the valid header
    if self.tf.frameExists("/odom_combined") and self.tf.frameExists(self.pose.header.frame_id):
#      t = self.tf.getLatestCommonTime("/odom_combined", self.pose.header.frame_id)
#      pos, quat = self.tf.lookupTransform("/odom_combined", self.pose.header.frame_id, t)
#      self.trans_pose.pose.orientation.x =
      self.trans_pose = self.tf.transformPose("/odom_combined", self.pose)
      self.trans_pose.pose.orientation.x = -self.trans_pose.pose.orientation.x
      self.trans_pose.pose.orientation.y = -self.trans_pose.pose.orientation.y


#    self.tf.waitForTransform('/odom_combined', self.pose.header.frame_id, rospy.Time(), rospy.Duration(2.0))
#    while not rospy.is_shutdown():
#      try:
#        now = rospy.Time.now()
#        self.tf.waitForTransform('/odom_combined', self.pose.header.frame_id, now, rospy.Duration(2.0))
#        (pos, quat) = self
        
#    self.trans_pose = self.tf.transformPose('/odom_combined', self.pose)
#    self.trans_pose.pose.orientation.x = -self.trans_pose.pose.orientation.x
#    self.trans_pose.pose.orientation.y = -self.trans_pose.pose.orientation.y
#        common_time = self.tf.getLatestCommonTime('/ar_marker_1', '/map')
#        self.pose.header.stamp = common_time
#        self.pose.header.frame_id = '/ar_marker_1'
#        self.pose.pose = pose
#        self.pose = self.tf.transformPose('/map', self.pose)
         

  def service_cb(self, dummy):
    rospy.logdebug('Returning trans_pose ' + str(self.pose))
    return srv.DestinationServiceResponse(pose = self.pose, trans_pose = self.trans_pose)

if __name__=='__main__':
    rospy.init_node('catch_me_destination_service_node')
    s = DestinationService()
    rospy.spin()
 

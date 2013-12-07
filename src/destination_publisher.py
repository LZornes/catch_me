#!/usr/bin/env python

import rospy
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker

class DestinationPublisher:
  def __init__(self):
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_cb)
    self.dest_pub = rospy.Publisher('catch_me_destination_publisher', AlvarMarker)

  def marker_cb(self, pose_markers):
    for i in range(0, len(pose_markers.markers)):
      marker = pose_markers.markers[i]
      #get marker 1
      if marker.id == 1:
        self.dest_pub.publish(marker)

if __name__=='__main__':
    rospy.init_node('catch_me_destination_publisher_node')
    s = DestinationPublisher()
    rospy.spin()

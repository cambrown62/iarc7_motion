#!/usr/bin/env python

# A helper class for a task that will spit out a velocity that's appropriate
# to reach and hold a desired position in the z dimension

import math
import rospy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class HeightHolder():
    def __init__(self):

        self._k_z = .1
        self._odometry = None
        self._current_velocity_sub = rospy.Subscriber('/odometry/filtered',
                                              Odometry,
                                              self._current_velocity_callback)
        try:
            self._TRACK_HEIGHT = rospy.get_param('~track_roomba_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

        while self._odometry is None:
            pass

        current_z = self._odometry.pose.pose.position.z

        self._last_vel_z = None

    def get_height_hold_response(self):

        delta_z = self._TRACK_HEIGHT - self._odometry.pose.pose.position.z 
        response = self._k_z * delta_z + self._odometry.twist.twist.linear.z
        return response

    def _current_velocity_callback(self, odometry):
        self._odometry = odometry
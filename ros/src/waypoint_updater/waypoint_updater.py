#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import waypoint_helper

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

MPH_TO_MPS = 0.44704
MAX_SPEED = 10.0 * MPH_TO_MPS #: Vehicle speed limit
LOOKAHEAD_WPS = 30  #: Number of waypoints we will publish


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.waypoint_index_pub = rospy.Publisher('waypoint_index', Int32, queue_size=1)

        # TODO: Add other member variables you need below

        self.base_waypoints = None
        self.pose = None  #: Current vehicle location + orientation
        self.frame_id = None

        self.loop()

    def loop(self):
        """ Publishes car index and subset of waypoints with target velocities """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.base_waypoints is None or self.pose is None or self.frame_id is None:
                continue

            # Where in base waypoints list the car is
            waypoint_index = waypoint_helper.get_closest_waypoint_index(self.pose, self.base_waypoints)

            # Ask to cruise or brake car
            target_speed = MAX_SPEED

            # Get subset waypoints ahead of vehicle and set target speeds
            lookahead_waypoints = waypoint_helper.get_next_waypoints(self.base_waypoints,
                                                                     waypoint_index, LOOKAHEAD_WPS)
            for waypoint in lookahead_waypoints:
                waypoint.twist.twist.linear.x = target_speed

            # Publish
            lane = waypoint_helper.make_lane_object(self.frame_id, lookahead_waypoints)
            self.final_waypoints_pub.publish(lane)
            self.waypoint_index_pub.publish(waypoint_index)

    def pose_cb(self, msg):
        self.pose = msg.pose  # store location (x, y)
        self.frame_id = msg.header.frame_id

    def waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

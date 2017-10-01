#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray

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

        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/traffic_light_state', Int32, self.traffic_state_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.waypoint_index_pub = rospy.Publisher('waypoint_index', Int32, queue_size=1)

        # TODO: Add other member variables you need below

        self.base_waypoints = None
        self.pose = None  #: Current vehicle location + orientation
        self.frame_id = None

        self.traffic_light_data = None
        self.traffic_light_state = None

        self.loop()

    def loop(self):
        """ Publishes car index and subset of waypoints with target velocities """
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.base_waypoints is None or self.pose is None or self.frame_id is None:
                continue

            if self.traffic_light_data is None:
                continue

            # Where in base waypoints list the car is
            waypoint_index = waypoint_helper.get_closest_waypoint_index(self.pose, self.base_waypoints)

            closest_traffic_light_index = None
            min_diff = 9999
            for index, light in enumerate(self.traffic_light_data.lights):
                is_ahead = waypoint_helper.is_waypoint_front(self.pose, light)
                if is_ahead:
                    traffic_light_index = \
                        waypoint_helper.get_closest_waypoint_index(light.pose.pose, self.base_waypoints)
                    diff = traffic_light_index - waypoint_index
                    if diff < min_diff:
                        min_diff = diff
                        closest_traffic_light_index = index

            #print closest_traffic_light_index

            stop_distance = 0;

            if closest_traffic_light_index is not None:
                traffic_light_pose = self.traffic_light_data.lights[closest_traffic_light_index].pose.pose
            # traffic_light_state = self.traffic_light_data.lights[closest_traffic_light_index].state

                stop_distance = waypoint_helper.get_distance(self.pose.position, traffic_light_pose.position)
            # print (stop_distance)

            # Distance from light to stop line is 25

            go_flag = False

            if self.traffic_light_state is not None:
                print(self.traffic_light_state.data)

                if self.traffic_light_state.data == 1 or self.traffic_light_state.data == 2:
                    #print("go_flag", go_flag)
                    go_flag = True

            if go_flag is False and 20 < stop_distance < 35:
            # if traffic_light_state != 2 and 20 < stop_distance < 35:
                # Get subset waypoints ahead of vehicle and set target speeds
                lookahead_waypoints = waypoint_helper.get_next_waypoints(self.base_waypoints,
                                                                         waypoint_index, LOOKAHEAD_WPS)
                for waypoint in lookahead_waypoints:
                    waypoint.twist.twist.linear.x = 0
            else:
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
        self.traffic_light_data = msg
        #print("self.traffic_light_data", self.traffic_light_data)
        pass

    def traffic_state_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_state = msg
        #print("self.traffic_light_state", self.traffic_light_state)
        

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

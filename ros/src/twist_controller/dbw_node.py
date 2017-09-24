#!/usr/bin/env python
import os
import csv
import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
import numpy as np
import math

from longitudinal_control import LongitudinalController
from lateral_control import LateralController


import dbw_helper

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

SAMPLE_RATE = 10.0 #Hz
DELAY = 0.035 #s

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)   # for steering wheel

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoint_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        # output file for debug purpose
        # base_path = os.path.dirname(os.path.abspath(__file__))
        # self.steerfile = os.path.join(base_path, 'steers.csv')
        # self.speedfile = os.path.join(base_path, 'speed.csv')
        #
        # self.steer_data = []
        # self.speed_data = []

        self.longitudinal_control = LongitudinalController(vehicle_mass,
                                                           wheel_radius,
                                                           accel_limit,
                                                           decel_limit,
                                                           DELAY)
        self.lateral_control = LateralController(vehicle_mass, wheel_base, steer_ratio,
                                                 max_lat_accel, max_steer_angle, 0, 5, DELAY)

        self.dbw_enabled = False
        self.waypoints = None
        self.pose = None
        self.velocity = None
        self.yawrate = None
        self.current_command = None
        self.frame_id = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(SAMPLE_RATE) # 50Hz
        sample_time = 1.0/SAMPLE_RATE
        self.longitudinal_control.set_sample_time(sample_time)
        self.lateral_control.set_sample_time(sample_time)

        while not rospy.is_shutdown():
            if self.waypoints is None \
                    or self.pose is None \
                    or self.velocity is None:
                continue

            close_way_point_id = dbw_helper.get_closest_waypoint_index(self.pose, self.waypoints)
            ref_spd_raw = self.waypoints[close_way_point_id].twist.twist.linear.x
            waypoint_coefficient = self.lateral_control.set_waypoint_coeff(self.pose, self.waypoints, self.velocity,
                                                                           polynomial_order=3,
                                                                           points_to_fit=20)

            sample_time_displacement = sample_time*ref_spd_raw
            radius_evaluation_point = np.array([0.4*sample_time_displacement,
                                                0.5*sample_time_displacement,
                                                0.6*sample_time_displacement])
            current_radius = dbw_helper.calculateRCurve(waypoint_coefficient, radius_evaluation_point)
            max_ref_spd = self.lateral_control.get_max_ref_speed(np.average(current_radius))

            ref_spd = ref_spd_raw
            if ref_spd > max_ref_spd:
                ref_spd = max_ref_spd

            throttle, brake = self.longitudinal_control.control_lqr(ref_spd, self.velocity, self.dbw_enabled)
            steer, cte_distance, cte_yaw = self.lateral_control.control_lqr(self.dbw_enabled)

            # output file for debug purpose
            #
            # fieldnames_steer = ['proposed', 'cte_distance', 'cte_yaw']
            # fieldnames_speed = ['ref_speed_raw', 'max_ref_spd','ref_speed','current_speed']
            #
            # self.speed_data.append({'ref_speed_raw': ref_spd_raw,
            #                         'max_ref_spd': max_ref_spd,
            #                         'ref_speed': ref_spd,
            #                         'current_speed': self.velocity})
            #
            # with open(self.speedfile, 'w') as csvfile:
            #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames_speed)
            #     writer.writeheader()
            #     writer.writerows(self.speed_data)
            #
            # self.steer_data.append({'proposed': steer,
            #                         'cte_distance': cte_distance,
            #                         'cte_yaw': cte_yaw})
            #
            # with open(self.steerfile, 'w') as csvfile:
            #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames_steer)
            #     writer.writeheader()
            #     writer.writerows(self.steer_data)

            if self.dbw_enabled:
                self.publish(throttle, brake, steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def twist_cmd_cb(self, msg):
        self.current_command = msg.twist

    def velocity_cb(self, msg):
        self.velocity = msg.twist.linear.x
        self.yawrate = msg.twist.angular.z

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = bool(msg.data)

    def waypoint_cb(self, msg):
        self.waypoints = msg.waypoints

    def pose_cb(self, msg):
        self.pose = msg.pose
        self.frame_id = msg.header.frame_id


if __name__ == '__main__':
    DBWNode()


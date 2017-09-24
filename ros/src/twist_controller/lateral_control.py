import dbw_helper
import rospy
from pid import PID
import math
import numpy as np


class LateralController(object):
    def __init__(self, vehicle_mass_, wheel_base_, steer_ratio_, max_lat_accel_,
                 max_steer_angle_, min_speed_, evaluation_location_, delay_=0.):
        self.vehicle_mass = vehicle_mass_
        self.wheel_base = wheel_base_
        self.steer_ratio = steer_ratio_
        self.max_lat_accel = max_lat_accel_*0.8
        self.max_steer_angle_cmd = max_steer_angle_
        self.sample_time = None
        self.time_step = rospy.get_time()
        self.min_speed = min_speed_
        self.int_val = 0.0
        self.coefficients = None
        self.evaluation_location = evaluation_location_
        self.control_action_last = 0
        self.delay = delay_

    def set_sample_time(self, sample_time_):
        self.sample_time = sample_time_

    def get_angle(self, radius, ref_speed):

        min_radius = radius
        if abs(ref_speed) > 0.1:
            min_radius = ref_speed**2/self.max_lat_accel

        angle = math.atan(self.wheel_base / max(min_radius, radius)) * self.steer_ratio

        return angle

    def set_waypoint_coeff(self, pose, waypoints,
                           current_spd,
                           polynomial_order=3, points_to_fit=20):

        _, _, yaw = dbw_helper.get_Euler_Angle(pose)

        pose.position.x = pose.position.x+current_spd*self.delay*math.cos(yaw)
        pose.position.y = pose.position.y+current_spd*self.delay*math.sin(yaw)
        yaw = yaw+math.tan(self.control_action_last)*self.delay/self.wheel_base*current_spd

        self.coefficients = dbw_helper.fit_waypoints(pose, waypoints, yaw,
                                                     polynomial_order=polynomial_order,
                                                     points_to_fit=points_to_fit)
        return self.coefficients

    def get_max_ref_speed(self, radius):
        return np.sqrt(self.max_lat_accel*np.abs(radius))

    def control_pid(self, dbw_enabled):
        """
        use PID to calculate steering command
        Args:
            pose (object) : A pose object
            waypoints (list) : A list of waypoint objects
            dbw_enabled(bool): control enabled
        Returns:
            steering_cmd(float): desired steering wheel command [rad]
            cte_distance(float): distance to trajectory center [m]
            cte_yaw(float): yaw angle difference with trajectory [rad]
        """
        cte_distance, cte_yaw = dbw_helper.cte(self.coefficients,
                                               evaluation_locaiton=self.evaluation_location)

        controller = PID(kp=0.01, ki=0.000001, kd=0.005)

        if abs(cte_distance)>20:
            controller.reset()

        steering = -controller.step(cte_distance, self.sample_time)
        steering_cmd = steering

        if steering_cmd > self.max_steer_angle_cmd:
            steering_cmd = self.max_steer_angle_cmd
        if steering_cmd < -self.max_steer_angle_cmd:
            steering_cmd = -self.max_steer_angle_cmd

        if dbw_enabled:
            return steering_cmd, cte_distance, cte_yaw
        controller.reset()
        return 0, 0, 0

    def control_lqr(self, dbw_enabled):
        """
        use LQR to calculate state feedback with integrator for distance error
        base model: kinematic error model
        assume center of mass at center of wheel base
        Args:
            pose (object) : A pose object
            waypoints (list) : A list of waypoint objects
            dbw_enabled(bool): control enabled
        Returns:
            steering_cmd(float): desired steering wheel command [rad]
            cte_distance(float): distance to trajectory center [m]
            cte_yaw(float): yaw angle difference with trajectory [rad]
        """
        cte_distance, cte_yaw = dbw_helper.cte(self.coefficients,
                                               evaluation_locaiton=self.evaluation_location)

        k_y = 0.007039605032495
        k_y_int = 3.162277660168384e-04
        k_yaw = 0.625435892486132

        self.int_val = self.int_val + cte_distance*self.sample_time

        cte = k_y*cte_distance + k_yaw*cte_yaw + k_y_int*self.int_val

        steering = cte
        steering_cmd = math.atan(steering)*self.steer_ratio

        if steering_cmd > self.max_steer_angle_cmd:
            steering_cmd = self.max_steer_angle_cmd
        if steering_cmd < -self.max_steer_angle_cmd:
            steering_cmd = -self.max_steer_angle_cmd

        self.control_action_last = steering_cmd/self.steer_ratio

        if dbw_enabled:
            return steering_cmd, cte_distance, cte_yaw

        self.int_val = 0.0
        self.control_action_last = 0

        return 0, 0, 0

    def control_preview(self, dbw_enabled, ref_speed):
        """
        use LQR to calculate state feedback with integrator for distance error
        base model: kinematic error model
        assume center of mass at center of wheel base
        use reference longitudinal speed as nominal longitudinal speed to estimate preview curvature
        Args:
            pose (object) : A pose object
            waypoints (list) : A list of waypoint objects
            dbw_enabled(bool): control enabled
        Returns:
            steering_cmd(float): desired steering wheel command [rad]
            cte_distance(float): distance to trajectory center [m]
            cte_yaw(float): yaw angle difference with trajectory [rad]
        """
        evaluation_location_initial = 3
        look_ahead_sample = 5

        cte_distance, cte_yaw = dbw_helper.cte(self.coefficients,
                                               evaluation_locaiton=self.evaluation_location)

        look_ahead_location = np.zeros(look_ahead_sample)
        for time_index in range(look_ahead_sample):
            look_ahead_location[time_index] = evaluation_location_initial+time_index*self.sample_time*ref_speed

        radius = dbw_helper.calculateRCurve(self.coefficients, look_ahead_location)
        steering_feedforward_cmd = self.get_angle(np.average(radius), ref_speed)

        k_y = 0.007039605032495
        k_y_int = 3.162277660168384e-04
        k_yaw = 0.625435892486132

        self.int_val = self.int_val + cte_distance * self.sample_time

        cte = k_y * cte_distance + k_yaw * cte_yaw + k_y_int * self.int_val

        steering = cte
        steering_feedback_cmd = math.atan(steering) * self.steer_ratio

        steering_cmd = steering_feedforward_cmd + steering_feedback_cmd

        if steering_cmd > self.max_steer_angle_cmd:
            steering_cmd = self.max_steer_angle_cmd
        if steering_cmd < -self.max_steer_angle_cmd:
            steering_cmd = -self.max_steer_angle_cmd

        if dbw_enabled:
            return steering_cmd, cte_distance, cte_yaw

        self.int_val = 0.0

        return 0, 0, 0

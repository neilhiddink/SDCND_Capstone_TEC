import dbw_helper
import rospy
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class LateralController(object):
    def __init__(self, vehicle_mass_, wheel_base_, steer_ratio_, max_lat_accel_, max_steer_angle_):
        self.vehicle_mass = vehicle_mass_
        self.wheel_base = wheel_base_
        self.steer_ratio = steer_ratio_
        self.max_lat_accel = max_lat_accel_
        self.max_steer_angle = max_steer_angle_
        self.max_steer_angle_cmd = max_steer_angle_*steer_ratio_
        self.sample_time = None
        self.time_step = rospy.get_time()

    def set_sample_time(self, sample_time_):
        self.sample_time = sample_time_

    def control(self, pose, waypoints, dbw_enabled):
        new_time_step = rospy.get_time()
        time_diff = new_time_step - self.time_step
        if time_diff < 1e-6:
            time_diff = 1e-6
        cte_distance, cte_yaw = dbw_helper.cte(pose, waypoints, polynomial_order=3, evaluation_locaiton=5, points_to_fit=10)

        controller = PID(kp=0.015, ki=0.0000001, kd=0.01)

        if abs(cte_distance)>20:
            controller.reset()

        steering = -controller.step(cte_distance, self.sample_time)
        steering_cmd = steering

        if steering_cmd > self.max_steer_angle:
            steering_cmd = self.max_steer_angle
        if steering_cmd < -self.max_steer_angle:
            steering_cmd = -self.max_steer_angle

        self.time_step = new_time_step
        if dbw_enabled:
            return steering_cmd, cte_distance, cte_yaw
        controller.reset()
        return 0, 0, 0

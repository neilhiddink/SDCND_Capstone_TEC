import dbw_helper
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

    def set_sample_time(self, sample_time_):
        self.sample_time = sample_time_

    def control(self, pose, waypoints):
        cte_distance, cte_yaw = dbw_helper.cte(pose, waypoints)
        controller = PID(kp=100, ki=0.1, kd=10)
        steering = controller.step(cte_distance, self.sample_time)

        return steering

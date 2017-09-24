from pid import PID

class LongitudinalController(object):
    def __init__(self,
                 vehicle_mass_,
                 wheel_radius_,
                 accel_limit_,
                 decel_limit_,
                 delay_=0.):
        self.vehicle_mass = vehicle_mass_
        self.wheel_radius = wheel_radius_
        self.accel_limit = accel_limit_
        self.decel_limit = decel_limit_
        self.max_throttle_torque = accel_limit_ * vehicle_mass_ * wheel_radius_
        self.max_break_torque = abs(decel_limit_ * vehicle_mass_ * wheel_radius_)
        self.sample_time = None
        self.cv = 0.32      # rolling resistance
        self.cr = 0.01      # aerodynamics drag
        self.g = 9.8        # gravity
        self.frontal_area = 2.4
        self.air_density = 2.858
        self.control_action_last = 0
        self.delay = delay_

    def get_control_action_last(self):
        return self.control_action_last

    def set_sample_time(self, sample_time_):
        self.sample_time = sample_time_

    def control_pid(self, ref_spd, current_spd, dbw_enabled):
        """
        use PID to get throttle and brake
        Args:
            ref_spd(float): desired speed [m/s]
            current_spd(float): actual speed [m/s]
            dbw_enabled(bool): control enabled
        Returns:
            throttle(float): percentage throttle request
            brake(float): request absolute brake torque [Nm]
        """
        controller = PID(kp=100, ki=0.1, kd=10)
        speed_error = ref_spd - current_spd
        acceleration = controller.step(speed_error, self.sample_time)

        torque = self.vehicle_mass * acceleration * self.wheel_radius
        throttle, brake = 0, 0
        if torque > 0:
            # throttle is the percent of max torque applied
            throttle, brake = min(1.0, torque / self.max_throttle_torque), 0.0
        else:
            # brake is the torque we need to apply
            throttle, brake = 0.0, min(abs(torque), self.max_break_torque)

        if dbw_enabled:
            return throttle, brake
        controller.reset()
        return 0, 0

    def set_feed_forward_parameter(self, cv_=0.32, cr_=0.01, g_=9.8, frontal_area_=2.4, air_density_=2.858):
        self.cv = cv_  # rolling resistance
        self.cr = cr_  # aerodynamics drag
        self.g = g_  # gravity
        self.frontal_area = frontal_area_
        self.air_density = air_density_

    def control_lqr(self, ref_spd, current_spd, dbw_enabled):
        """
        use LQR to calculate state feedback with integrator
        augment state: [speed_error;error_integration]
        weight for state: [10000,1]
        weight for control: 1/10000
        reference speed is used as linearization point for aerodynamics drag
        nominal force is used as feedforward for reference speed
        Args:
            ref_spd(float): desired speed [m/s]
            current_spd(float): actual speed [m/s]
            dbw_enabled(bool): control enabled
        Returns:
            throttle(float): percentage throttle request
            brake(float): request absolute brake torque [Nm]
        """
        current_spd = current_spd+self.delay*self.control_action_last/self.vehicle_mass

        controller = PID(kp=1.052854431521342e+03, ki=31.622776601683810, kd=0)
        speed_error = ref_spd - current_spd

        rolling_resistance = self.vehicle_mass*self.g*self.cr
        aerodynamics_drag = 0.5 * self.air_density*self.cv*self.frontal_area*ref_spd*ref_spd
        force_feed_forward = rolling_resistance+aerodynamics_drag

        force_feed_back = controller.step(speed_error, self.sample_time)
        control_action_force = max(min((force_feed_back+force_feed_forward),self.accel_limit*self.vehicle_mass),
                                   self.decel_limit*self.vehicle_mass)
        self.control_action_last = control_action_force
        torque = control_action_force * self.wheel_radius

        throttle, brake = 0, 0
        if torque > 0:
            # throttle is the percent of max torque applied
            throttle, brake = min(1.0, torque / self.max_throttle_torque), 0.0
            self.control_action_last = 0

        else:
            # brake is the torque we need to apply
            throttle, brake = 0.0, min(abs(torque), self.max_break_torque)

        if dbw_enabled:
            return throttle, brake

        controller.reset()
        self.control_action_last = 0
        return 0, 0


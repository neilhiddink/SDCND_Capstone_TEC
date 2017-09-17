
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class LongitudinalController(object):
    def __init__(self,
                 vehicle_mass_,
                 wheel_radius_,
                 accel_limit_,
                 decel_limit_,
                 damping=50):
        self.vehicle_mass = vehicle_mass_
        self.wheel_radius = wheel_radius_
        self.accel_limit = accel_limit_
        self.decel_limit = decel_limit_
        self.max_throttle_torque = accel_limit_ * vehicle_mass_ * wheel_radius_
        self.max_break_torque = decel_limit_ * vehicle_mass_ * wheel_radius_
        self.damping = damping


    def control(self, ref_spd, current_spd):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer



        torque = self.vehicle_mass * acceleration * self.wheel_radius
        throttle, brake = 0, 0
        if torque > 0:
            # throttle is the percent of max torque applied
            throttle, brake = min(1.0, torque / self.max_throttle_torque), 0.0
        else:
            # brake is the torque we need to apply
            throttle, brake = 0.0, min(abs(torque), self.max_break_torque)

        return throttle, brake

from lowpass import LowPassFilter
from math import atan
import rospy
import numpy as np
from pid import PID
from math import atan

class SteeringAngleController():
    def __init__(self, wheel_base, steering_ratio, max_steering_angle, max_lateral_acc):
        self.wheel_base = wheel_base
        self.steering_ratio = steering_ratio
        self.max_lateral_acc = max_lateral_acc
        self.max_steering_angle = max_steering_angle
    
    def control(self, current_linear_velocity, target_linear_velocity, target_angular_velocity):
        velocity_ang = current_linear_velocity * target_angular_velocity / target_linear_velocity  if abs(target_linear_velocity) > 0. else 0.
        velocity_ang = np.clip(velocity_ang, -self.max_lateral_acc, self.max_lateral_acc)
        if velocity_ang == 0:
            angle = 0.0
        else:
            r = max(current_linear_velocity, 0.1) / velocity_ang
            angle = atan(self.wheel_base / r) * self.steering_ratio
            angle  = np.clip(angle, -self.max_steering_angle, self.max_steering_angle)
        

        return angle


class ThrottleBrakeController():
    def __init__(self, car_mass, wheel_radius, deceleration_limit):
        self.lowpass_filter = LowPassFilter(tau=0.5, ts=.02)
        self.car_mass = car_mass
        self.deceleration_limit = deceleration_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()

        kp = 0.3
        ki = 0.1
        kd = 0.0
        self.throttle_controller_pid = PID(kp, ki, kd)

    def control(self, current_speed, target_speed):
        current_speed = self.lowpass_filter.filter(current_speed) 

        error_vel = target_speed - current_speed

        self.last_velocity = current_speed

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller_pid.step(error_vel, sample_time)

        brake = 0.0

        if target_speed == 0.0 and current_speed < 0.1:
            throttle = 0
            brake = 700
        elif throttle < 0.1 and error_vel < 0.0:
            throttle = 0.0
            decel = max(error_vel, self.deceleration_limit)
            brake = min(700, abs(decel) * self.car_mass * self.wheel_radius)

        return throttle, brake 

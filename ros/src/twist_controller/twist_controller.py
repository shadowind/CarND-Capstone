from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        #get the vehicle specific parameters
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit'] 
        self.wheel_radius = kwargs['wheel_radius'] 
        self.wheel_base = kwargs['wheel_base'] 
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.min_speed = kwargs['min_speed']
        
        #init controllers
        self.pid = PID(0.4, 0.0, 0.02, self.decel_limit, self.accel_limit)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

    def reset(self):
        self.pid.reset()
         
    def control(self, *args, **kwargs):
        # get all control params
        new_linear_vel = kwargs[0]
        new_angular_vel = kwargs[1]
        cur_linear_vel = kwargs[2]
        cte = kwargs[3]
        duration = kwargs[4]
        acceleration_update = steering_update = brake_update = 0.0
        
        
        #get the strring angle update from yaw controller
        steering_update = self.yaw_controller.get_steering(new_linear_vel, new_angular_vel, cur_linear_vel)
        
        #get acceleration update from pid 
        acceleration_update = self.pid((new_linear_vel - cur_linear_vel), duration)
        
        #if we were to decelerate, calculate the break update
        if (acceleration_update <= 0.0 and -1.0*acceleration_update >= self.brake_deadband):
             brake_update = -1.0*acceleration_update * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius
            
        
        return acceleration_update, brake_update, steering_update

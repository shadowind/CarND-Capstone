import rospy
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
        self.pid_linear = PID(0.4, 0.0, 0.02, self.decel_limit, self.accel_limit)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        self.steer_filter = LowPassFilter(tau = 2, ts = 1)
        self.throttle_lilter = LowPassFilter(tau = 2, ts = 1)
        self.pid_steer = PID(0.12, 0.002, 0.1, -self.max_steer_angle, self.max_steer_angle)

    def reset(self):
        self.pid_linear.reset()
        self.pid_steer.reset()
         
    def control(self, *args, **kwargs):
        # get all control params
        new_linear_vel = args[0]
        new_angular_vel = args[1]
        cur_linear_vel = args[2]
        cte = args[3]
        duration = args[4]
        acceleration_update = steering_update = brake_update = 0.0
        
        
        #get the strring angle update from yaw controller
        steering_update = self.yaw_controller.get_steering(new_linear_vel, new_angular_vel, cur_linear_vel)
        #steering_update = self.steer_filter.filt(steering_update)
        steering_update += self.pid_steer.step(cte, duration)#use steer PID to correct the calculated steering update
        
        #get acceleration update from pid 
        acceleration_update = self.pid_linear.step((new_linear_vel - cur_linear_vel), duration)
        acceleration_update = self.throttle_lilter.filt(acceleration_update)
        
        #if we were to decelerate, calculate the break update
        if (acceleration_update < 0.0) :
            brake_update = -1.0*acceleration_update * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius
            acceleration_update = 0.0
        print("acceleration_update:", acceleration_update, " brake_update:", brake_update, " steering_update:", steering_update)  
        rospy.loginfo("new control info: throttle: %s, brake: %s, steering: %s" %(acceleration_update, brake_update, steering_update))
        return acceleration_update, brake_update, steering_update
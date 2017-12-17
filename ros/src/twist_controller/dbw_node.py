#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
import math
import numpy as np

from twist_controller import Controller

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

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_config = {
        'vehicle_mass': rospy.get_param('~vehicle_mass', 1736.35),
        'fuel_capacity' : rospy.get_param('~fuel_capacity', 13.5),
        'brake_deadband' : rospy.get_param('~brake_deadband', .1),
        'decel_limit' : rospy.get_param('~decel_limit', -5),
        'accel_limit' : rospy.get_param('~accel_limit', 1.),
        'wheel_radius' : rospy.get_param('~wheel_radius', 0.2413),
        'wheel_base' : rospy.get_param('~wheel_base', 2.8498),
        'steer_ratio' : rospy.get_param('~steer_ratio', 14.8),
        'max_lat_accel' : rospy.get_param('~max_lat_accel', 3.),
        'max_steer_angle' : rospy.get_param('~max_steer_angle', 8.),
        'min_speed' : rospy.get_param('~min_speed', 0.5)
        }

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Create `TwistController` object
        self.controller = Controller(**vehicle_config)
        
        self.prev_time_stamp = rospy.get_rostime()

        # Subscribe to all the topics need to
        self.new_vel = None
        self.twist_cmd_sub = rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cmd_callback)
        
        self.cur_vel = None
        self.vel_cmd_sub = rospy.Subscriber("/current_velocity", TwistStamped, self.cur_vel_cmd_callback)
        
        self.dbw_enabled = False
        self.dbw_status_sub = rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_status_callback)
        
        self.cur_pose = None
        self.cur_pose_sub = rospy.Subscriber("/current_pose", PoseStamped, self.cur_pose_callback)
        
        self.final_way_pts = None
        self.final_way_pts_sub = rospy.Subscriber("/final_waypoints", Lane, self.final_way_pts_callback)
        
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            if (self.cur_vel is not None) and (self.new_vel is not None) and (self.final_way_pts is not None):
                cur_time_stamp = rospy.get_rostime()
                duration = cur_time_stamp - self.prev_time_stamp
                duration_in_seconds = duration.secs + (1e-9 * duration.nsecs)
                self.prev_time_stamp = cur_time_stamp
                throttle, brake, steering = self.controller.control(self.new_vel.twist.linear.x,self.new_vel.twist.angular.z,self.cur_vel.twist.linear.x,self.get_cte(),duration_in_seconds)
                if not self.dbw_enabled:
                    #reset contoller
                    self.controller.reset()
                if self.dbw_enabled:
                    self.publish(throttle, brake, steering)
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
    

    def twist_cmd_callback(self, data):
        self.new_vel = data
        
    def cur_vel_cmd_callback(self, data):
	    self.cur_vel = data
	    
    def dbw_status_callback(self, data):
	    self.dbw_enabled = data.data
	
    def cur_pose_callback(self, data):
	    self.cur_pose = data

    def final_way_pts_callback(self, data):
        self.final_way_pts = data.waypoints
        
    def get_cte(self):
        #transform way points and car pose into cars co-coordinate system
        origin = self.final_way_pts[0].pose.pose.position
        
        #Shift way points
        xy_way_pts = list(map(lambda way_pt: [way_pt.pose.pose.position.x, way_pt.pose.pose.position.y], self.final_way_pts))
        shifted_way_pts = xy_way_pts - np.array([origin.x, origin.y])
        
        #Rotate way points
        rotatation_angle = np.arctan2(shifted_way_pts[10, 1], shifted_way_pts[10, 0])#take 10 points to calculate angle
        rotation_matrix = np.array([[np.cos(rotatation_angle), -np.sin(rotatation_angle)], [np.sin(rotatation_angle), np.cos(rotatation_angle)]])
        transformed_way_pts = np.dot(shifted_way_pts, rotation_matrix)
        
        #Shift car pose
        shifted_car_pose = np.array([self.cur_pose.pose.position.x - origin.x, self.cur_pose.pose.position.y - origin.y])
        
        #Rotate car pose
        rotated_car_pose = np.dot(shifted_car_pose, rotation_matrix)
        
        #Find CTE
        #Fit a 2 degree polynomial through the transformed way points to get the y(cte) value of the car
        coefficients = np.polyfit(transformed_way_pts[:, 0], transformed_way_pts[:, 1], 2)
        
        expected_y_val = np.polyval(coefficients, rotated_car_pose[0])
        actual_y_val = rotated_car_pose[1]
        
        return (expected_y_val - actual_y_val)

if __name__ == '__main__':
    DBWNode()

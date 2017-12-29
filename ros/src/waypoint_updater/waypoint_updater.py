#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped,TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo('Node waypoint_updated started.')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # Load waypoint only once
        self.load_current_pose = False
        # Store the current pose as "PoseStamped" type
        self.current_pose_msg = PoseStamped()
        # Store the current waypoints for only once as "Lane" type
        self.base_waypoints = Lane()
        # The index of closest current waypoints
        self.current_idx = -1
        self.last_closest_waypoint_idx = -1
        self.received_waypoints = False
        # Speed limit stored in '/waypoint_loader/velocity' as km/h, for velocity in waypoints data, the unit is m/s, so make the calculation here
        self.speed_limit = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600
        
        #Traffic light related
        self.cur_red_light_wp_idx = None

        self.loop()
        # rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        if not self.load_current_pose:
            # rospy.loginfo("Logging position data, curret_pose. x: %s,y: %s, z: %s" % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
            self.load_current_pose = True

        # Update current pose msg
        self.current_pose_msg = msg
                # pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints.waypoints = waypoints.waypoints
        self.received_waypoints = True
        # pass

    def velocity_cb(self, msg):
        # rospy.loginfo(msg)
        self.current_velocity_x = msg.twist.linear.x
        #print "Car actual V:", self.current_velocity_x

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        #rospy.loginfo("msg.data = %s"% (msg.data))
        if self.cur_red_light_wp_idx != msg.data:
            self.cur_red_light_wp_idx = msg.data if msg.data >=0 else None
            #rospy.loginfo("cur_red_light_wp_idx = %s"% (self.cur_red_light_wp_idx))
            #update the new WP considering RED light
            self.publish_waypoints()
        

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def publish_waypoints(self):
        # TODO : This function publishes the next waypoints
        if self.received_waypoints and self.load_current_pose:
            next_wp_idx = self.get_next_waypoint()
            #rospy.loginfo("next_wp_idx = %s"% (next_wp_idx))
            array_final_waypoints = Lane()
            red_light_idx_in_final_waypoints = None
            for idx_waypt in range(LOOKAHEAD_WPS):
                # Check if drive more than one round
                idx_waypt_to_appen
                d = (next_wp_idx + idx_waypt) % len(self.base_waypoints.waypoints)

                #Get the information about the waypoint that we will append
                waypt_to_append = self.base_waypoints.waypoints[idx_waypt_to_append]
                # print "TARGET SPEED for each point:", self.get_waypoint_velocity(waypt_to_append)
                # Append the waypoint to the array of waypoints.
                array_final_waypoints.waypoints.append(waypt_to_append)
                
                #rospy.loginfo("self.cur_red_light_wp_idx = %s, idx_waypt_to_append = %s"% (self.cur_red_light_wp_idx, idx_waypt_to_append))
                if(self.cur_red_light_wp_idx and (self.cur_red_light_wp_idx == idx_waypt_to_append)):
                    red_light_idx_in_final_waypoints = idx_waypt
                    
            #for i in range(5):
            #    print "-------Index:", i," ;target velocity:", self.get_waypoint_velocity(array_final_waypoints.waypoints[i])
           
            #Once the final waypoints is built, check for red-lights and reduce the velocieties of way points to gracefully halt the vehicle at red light
            if(red_light_idx_in_final_waypoints):
                total_dist_to_red_light = self.distance(array_final_waypoints.waypoints, 0, red_light_idx_in_final_waypoints)
                rospy.loginfo("Number of wps to stopline %s"%(red_light_idx_in_final_waypoints))
                for i in range(len(array_final_waypoints.waypoints)):
                    if (i < red_light_idx_in_final_waypoints-13):
                    #reduce the velocity
                        dist = self.distance(array_final_waypoints.waypoints, i, red_light_idx_in_final_waypoints)#distance to red light
                        decel = 1 # decelerate at 1 m2/s2
                        vel = math.sqrt(2*decel*dist) #based on distance, calcualte velocity and set it in waypoint
                        if (vel < self.get_waypoint_velocity(array_final_waypoints.waypoints[i])):
                            self.set_waypoint_velocity(array_final_waypoints.waypoints, i, vel) #set velocity
                    else:
                        self.set_waypoint_velocity(array_final_waypoints.waypoints, i, 0)
            else:
                for i in range(len(array_final_waypoints.waypoints)):
                    self.set_waypoint_velocity(array_final_waypoints.waypoints, i, self.speed_limit)           
            # for i in range(5):
            #    print "-------Index:", i," ;target velocity:", self.get_waypoint_velocity(array_final_waypoints.waypoints[i])
            #TEST START
            #if(red_light_idx_in_final_waypoints == 1):
            #    self.cur_red_light_wp_idx = (next_wp_idx + 200) % len(self.base_waypoints.waypoints)
            #TEST END
              
            # Publish the Lane info to the /final_waypoints topic
            self.final_waypoints_pub.publish(array_final_waypoints)

    def get_next_waypoint(self):
        # Part 1: find the nearst waypoints. 
        ego_pose = self.current_pose_msg.pose
        waypoints = self.base_waypoints.waypoints
        closest_wp_dist = 10000.0
        closest_wp_idx = -1
        next_wp_idx = -1
        for idx, waypt in enumerate(waypoints):
            delta_x = ego_pose.position.x - waypt.pose.pose.position.x
            delta_y = ego_pose.position.y - waypt.pose.pose.position.y
            delta_z = ego_pose.position.z - waypt.pose.pose.position.z
            dist_to_wp = math.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z)

            if (dist_to_wp < closest_wp_dist):
                closest_wp_dist = dist_to_wp
                closest_wp_idx = idx
        if(closest_wp_idx>0):
            # rospy.loginfo("Logging closest waypoint x: %s,y: %s, z: %s" % (waypoints[closest_wp_idx].pose.pose.position.x, waypoints[closest_wp_idx].pose.pose.position.x, waypoints[closest_wp_idx].pose.pose.position.z))
            # rospy.loginfo("Closest waypoint index:%d" % (closest_wp_idx))
            next_wp_idx = closest_wp_idx
        # Part 2: Check if the nearest point is ahead or behind the car ego pose.
        # 1. Get the car current orientation in Euler angle(yaw/heading) by converting from quaternions
            quaternion = (self.current_pose_msg.pose.orientation.x, self.current_pose_msg.pose.orientation.y, self.current_pose_msg.pose.orientation.z, self.current_pose_msg.pose.orientation.w)
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        # 2. get the difference between two angles ro determine whether a car is behind or ahead of waypoint
            map_x = waypoints[closest_wp_idx].pose.pose.position.x
            map_y = waypoints[closest_wp_idx].pose.pose.position.y
            heading = math.atan2((map_y - ego_pose.position.y), (map_x - ego_pose.position.x))
            angle = abs(yaw - heading)
            if angle > (math.pi / 4):
                next_wp_idx += 1


            # rospy.loginfo("current orientation: %s" % (yaw))

        return next_wp_idx

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def loop(self):
        rate = rospy.Rate(10)  # 50Hz
        while not rospy.is_shutdown():
            self.publish_waypoints()
            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
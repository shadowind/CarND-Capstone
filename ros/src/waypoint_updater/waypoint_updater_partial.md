#### Run the controller

1. Make the controller
>cd ~/Documents/CarND/Term3/CarND-Capstone/ros
>catkin_make

2. In a new terminal window, start roscore
>roscore

3. Start the simulator, select screen resolution 800x600, click SELECT, uncheck the Manual checkbox. Ideally, run the simulator in the host environment (outside of the virtual machine).
>cd ~/Downloads/linux_sys_int/
>./sys_int.x86_64 
4. In a new terminal window, start the controller
>cd system-integration/ros
>source devel/setup.sh
>roslaunch launch/styx.launch

#### Callback function
A callback function is a function which is:
回调函数是一个作为参数传递给另外一个函数的函数，它在主体函数执行过程中的特定条件下执行。
- ssed as an argument to another function, and,
- invoked after some kind of event.  

Details click [here](http://www.jianshu.com/p/28481090ae94)


approaching TL in 50 points 


#### Output log to screen
Add output="screen" cwd="node"
```<node pkg="tl_detector" type="tl_detector.py" name="tl_detector" output="screen" cwd="node">```

#### Find orientation of waypoints 
https://discussions.udacity.com/t/what-is-the-meaning-of-the-various-waypoint-fields/406030/2
the topic ‘/current_pose’ also gives you its current orientation as a quaternion. You probably have to convert that to Euler angles. With these you can check with trigonometry whether the car is facing the waypoint or if the waypoint is behind.
Understanding Wusyrtnions: https://www.youtube.com/watch?v=3BR8tK-LuB0
unit of speed in /current_velocity twist.twist.linear.x  m/s


What need to do for way_point updater
https://discussions.udacity.com/t/do-i-understand-waypoint-updater-correctly/449541/7

you send to final_waypoints a list of waypoints (200, by default) with a speed that increses from waypoint to waypoint, with an acceleration you decide. If you determine the car can reach cruise speed within those 200 waypoints, then from that given waypoint they all have constant speed equal to the cruise speed.


speed_limit = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600. # m/s


linear_velocity the wanted linear velocity for the car, as set in the waypoints published to /final_waypoints;
angular_velocity the wanted angular velocity, again as set in the waypoints;
current_velocity the current car linear velocity, as reported by the /current_velocity topic.


## Transform Quaternion to Euler Angle

this part is particular hard for me to understand, intially wiki explaination doesn't make sense.

The Euler angles are three angles introduced by Leonhard Euler to describe the orientation of a rigid body with respect to a fixed coordinate system
\alpha  (or {\displaystyle \varphi } \varphi ) is the angle between the x axis and the N axis (x-convention - it could also be defined between y and N, called y-convention).
{\displaystyle \beta } \beta  (or {\displaystyle \theta \,} \theta \,) is the angle between the z axis and the Z axis.
{\displaystyle \gamma } \gamma  (or {\displaystyle \psi } \psi ) is the angle between the N axis and the X axis (x-convention).
\to
for α and γ, the range is defined modulo 2π radians. For instance, a valid range could be [−π, π).
for β, the range covers π radians (but can't be said to be modulo π). For example, it could be [0, π] or [−π/2, π/2].
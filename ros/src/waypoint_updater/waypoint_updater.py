#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

ASSUMPTION: 

'''

# The initial value of waypoints we will publish every time. It will be doubled
# when the value is not suitable
LOOKAHEAD_WPS_INIT = 100
MAX_DECEL = 1.0
MAX_ACCEL = 1.0
LOOP = False
# rate frequency the loops spin in ROS
PUBLISH_RATE = 2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

	#### Define Member Variables ####
	
	# current pose of the vehicle, composed of position and orientation
	self.current_pose = None 
	# list of base waypoints, of type Lane.waypoints
	self.base_waypoints_list = None
	# total number of base waypoints
	self.num_base_waypoints = None
	# list of current waypoints, which will be updated regularly. 
	# Of type Lane.waypoints
	self.current_waypoints_list = None 
	# a cursor recording the next start position of waypoints for publishing
	self.waypoint_cursor = None
	# sequence number of published message
	self.msg_seq_num = 0
	# stop line index (range: [-1, self.num_base_waypoints-1])
	# -1 means no stop
	self.stop_line_index = -1
	# the allowed maximal velocity 
	## We possibly need to recalculate this if we wish to give test on test lot
	self.max_velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))
	# previous traffic status, it can be 
	#	None : not clear
	#	True : Red/Yellow
	#	False: Green
	self.previous_state = None
	# This is the real lookahead value we are going to use for publishing
	self.LOOKAHEAD_WPS = LOOKAHEAD_WPS_INIT
	# flag up if LOOP is True and the car is approaching the destination
	self.arriving = False
	self.destination = None

	self.current_velocity = None
	rospy.Subscriber('/current_velocity', TwistStamped, self.callback_current_velocity)


	#### Subscriber and Publisher ####
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
	# TODO: Add a subscriber for /obstacle_waypoint below if it is necessary

	# set queue_siye to 1 so that the newest message is alway be used
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

	#### Main Part ####
	rate = rospy.Rate(PUBLISH_RATE)
	#n = 0
	while not rospy.is_shutdown():
	    #rospy.logwarn('Updating Round %s', n)
	    if self.base_waypoints_list is not None and self.current_pose is not None:   
	        self.update_current_waypoints_list()
		self.publish(self.current_waypoints_list)
		#self.print_velocities()
	    #n += 1
	    #rospy.logwarn('GO TO SLEEP\n')
	    rate.sleep()


    def update_current_waypoints_list(self):
	'''
	To execute this step, make sure neither self.base_waypoints_list, nor 
	self.current_pose is None.

	update the current waypoints list self.current_waypoints_list
	'''

	#### 1. Update Current Waypoints List####	
	if self.current_waypoints_list is None:
	    self.update_from_base_waypoints()
	else:
	    closest_id = self.get_closest_waypoint_index(self.current_waypoints_list)
	    if closest_id == self.LOOKAHEAD_WPS:
		# meaning the current waypoints was used up with current velocity
		rospy.logwarn('DANGER! SLOW DOWN!')
		self.LOOKAHEAD_WPS *= 2
		rospy.logwarn('Parameter self.LOOKAHEAD_WPS is doubled: %s', self.LOOKAHEAD_WPS)
		self.update_from_base_waypoints()	
	    elif closest_id is not 0:
		self.update_from_current_waypoints(closest_id)

	#### 2. Ajust Stop Line According To ARRIVING&LOOP#####
	stop_line = self.stop_line_index
	if self.arriving:	
	    stop_line = self.destination if stop_line < 0 else min(stop_line, self.destination)
	    #rospy.logwarn('Arriving:stop_line = %s', stop_line)
	#rospy.logwarn('self.stop_line_index = %s', stop_line)

	#### 3. Update Velocities of Current Waypoints####
	if stop_line < 0:
	    #rospy.logwarn('>>>GREEN LIGHT or NONE<<<')
	    self.accelerate(acceleration=1.0)
	    #self.previous_state = False
	else: 
	    # present: Red/Yellow light detected
	    if self.arriving:
		stop_index = self.LOOKAHEAD_WPS-(self.num_base_waypoints+self.waypoint_cursor-stop_line)
	    else:
	        stop_index = self.LOOKAHEAD_WPS-(self.waypoint_cursor-stop_line)
	    #rospy.logwarn('stop_index = %s', stop_index)
	    if 0 <= stop_index < self.LOOKAHEAD_WPS:
		#rospy.logwarn('decelerate is required to use')
	        self.decelerate(stop_index)
	        #self.previous_state = True
	    elif stop_index >= self.LOOKAHEAD_WPS:
		#rospy.logwarn('stop_index >= self.LOOKAHEAD_WPS')
	        # red/yellow light was detected, but within a safe distance. 
		self.accelerate(acceleration=0.5) 
		#self.previous_state = False
	    else:
		#rospy.logwarn('stop_index < 0')
		if self.arriving:
		    # stop anyway
		    return		
		self.accelerate(acceleration=1.0)
	        #self.previous_state = False
	if self.arriving:
	    self.print_velocities()


    def update_from_current_waypoints(self, closest_id):
	'''
	If self.current_waypoints_list is not None and the next starting position
	is less than self.LOOKAHEAD_WPS, then this function is used.
	'''
	# the next position of self.waypoint_cursor
	to_index = self.waypoint_cursor + closest_id
	if to_index >= self.num_base_waypoints:
	    if not LOOP:
		#rospy.logwarn('ARRIVING!')
		self.arriving = True
	        self.destination = self.num_base_waypoints - 1 
	    del self.current_waypoints_list[:closest_id]
	    self.current_waypoints_list.extend(self.base_waypoints_list[self.waypoint_cursor:self.num_base_waypoints])
	    to_index %= self.num_base_waypoints 
	    self.current_waypoints_list.extend(self.base_waypoints_list[0:to_index])
	    self.waypoint_cursor = to_index 
	else:
	    del self.current_waypoints_list[:closest_id]
	    self.current_waypoints_list.extend(self.base_waypoints_list[self.waypoint_cursor:to_index])
	    self.waypoint_cursor = to_index

    def update_from_base_waypoints(self):
	''' 
	This is used to intialize the self.current_waypoints_list from
	self.base_waypoints_list
	Scenarios: self.current_waypoints_list is None, or
		   self.current_waypoints_list is not None, but next starting
		   position is self.LOOKAHEAD_WPS.
	'''
	# get the closest position from self.base_waypoints_list
	closest_id = self.get_closest_waypoint_index(self.base_waypoints_list)
	# define waypoint cursor by self.LOOKAHEAD_WPS
	self.waypoint_cursor = closest_id + self.LOOKAHEAD_WPS
	# truncates the cursor if necessary
	#if self.waypoint_cursor >= self.num_base_waypoints:
	#    self.waypoint_cursor = self.num_base_waypoints
	# initialize self.current_waypoints_list
	self.current_waypoints_list = self.base_waypoints_list[closest_id:self.waypoint_cursor]

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def publish(self, waypoints_list):
	'''
	Adapted from waypoint_follower.py. It shoulde be OK
	'''
	lane = Lane()
	lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.header.seq = self.msg_seq_num
        lane.waypoints = waypoints_list
        self.msg_seq_num += 1
        self.final_waypoints_pub.publish(lane)

    def accelerate(self, acceleration=MAX_ACCEL):
	'''
	This function is used in acceleration.

	We will acclerate the velocities for self.current_waypoints_list base on
	the velocity of the first front waypoints of the car and its current
	velocity. Meanwhile, the updated velocities will be smaller than 
	self.max_velocity which is abtained from /waypoint_loader/velocity.

	Formular: set acceleration = MAX_ACCEL < 10 m/s**2
		  v(t)**2 = 2*a*S(t) + v(0)**2

	Note: /dbw_node/accel_limit: 1.0????
	'''
	#rospy.logwarn('Accelerate is used')
	waypoint_prev = self.current_waypoints_list[0]
	v0 = self.get_waypoint_velocity(waypoint_prev)
	if v0 == 0 or self.current_velocity < 0.5:
	#if v0 == 0:
	    v0 = 1.0
	    self.set_waypoint_velocity(self.current_waypoints_list, 0, v0)
	dist_prev = 0.0
	for i in range(1, len(self.current_waypoints_list)):
	    waypoint = self.current_waypoints_list[i]
	    dist_prev += self.distance_euclid(waypoint.pose.pose.position, waypoint_prev.pose.pose.position)
	    waypoint_prev = waypoint
	    vi = math.sqrt(2*acceleration*dist_prev + v0**2)
	    vi = min(self.max_velocity, vi)
	    self.set_waypoint_velocity(self.current_waypoints_list, i, vi)

    def decelerate(self, stop_index):
	'''
	This function is used in deceleration.

	Assumption: self.current_waypoints_list has been updated and currently
		    self.stop_line_index >= 0 (Red/Yellow light detected)
	
	In this case, we decelerate the car based on the velocity of the first 
	waypoint in front of the car.

	Fact: in general, the maximal deceleration of a car is in 6 to 8 m/s**2
	the average deceleration locates in 3-4 m/s**2
	we control the deceleration so that it is in 1.5-2.5 m/s**2 in order to 
	produce a better experience.

	:param stop_index: a positive integer between 0 and self.LOOKAHEAD_WPS-1.

	Note: /dbw_node/accel_limit: 1.0
	      /dbw_node/decel_limit: -5.0
	'''
	#rospy.logwarn('Decelerate is used')
	# save the velocity of the first waypoint
	v0 = self.get_waypoint_velocity(self.current_waypoints_list[0])
	# calculate the distance from first waypoint to stop_index waypoint
	S = self.distance(self.current_waypoints_list, 0, stop_index)
	#if S < 2.0: 
	    # if the car is close enough to the stop line, stop it any away
	    #for i in range(len(self.current_waypoints_list)):
       	        #self.set_waypoint_velocity(self.current_waypoints_list, i, 0.0)
	#if S < self.current_velocity**2/(2.0*4.0):
	    # here I assume decel_limit=-4.0
	    #rospy.logwarn('My God! I cannt STOP in time. Let\'s ACCELERATE.')
	    #self.accelerate()
	if S < v0**2/(2.0*4.5):
	    # here I assume decel_limit=-4.5
	    # and I changed self.current_velocity to v0 because all control commands
	    # are based on predefined waypoints velocities
	    #rospy.logwarn('My God! I cannt STOP in time. Let\'s ACCELERATE.')
	    self.accelerate()
	#elif S < 2.0 and v0 < 2.0:
	elif S < 2.0:
	    for i in range(len(self.current_waypoints_list)):
       	        self.set_waypoint_velocity(self.current_waypoints_list, i, 0.0)
	else:
	    # calculate deceleration by supposing the car slow down from the very
	    # beginning
	    deceleration = max(v0**2/(2*S), 0.2)
	    #rospy.logwarn('	deceleration is %s', deceleration)
	    # adjust max(,0.5) to max(,0.2) to make the velocities of last several 
	    # waypoints to 0.

	    self.decelerate_from_beginning(stop_index, deceleration)

    def decelerate_from_beginning(self, stop_index, deceleration):
	'''
	Make sure you deliver a proper deceleration value. We won't check the
	velocities so that they are less than self.max_velocity.
	'''
	deceleration = abs(deceleration)
	waypoint_prev = self.current_waypoints_list[stop_index]
	self.set_waypoint_velocity(self.current_waypoints_list, stop_index, 0.0)
	# set the velocities of waypoints with indexes [0,...,stop_index-1]
	dist_prev = 0.0
	for i in range(stop_index-1, -1, -1):
	    waypoint = self.current_waypoints_list[i]
	    dist_prev += self.distance_euclid(waypoint.pose.pose.position, waypoint_prev.pose.pose.position)
	    vi = math.sqrt(2*deceleration*dist_prev)
	    if vi < 1.0:
		vi = 0.0
	    self.set_waypoint_velocity(self.current_waypoints_list, i, vi)
	    waypoint_prev = waypoint
	# v0 should be less than self.max_velocity.
	if self.get_waypoint_velocity(self.current_waypoints_list[0]) > self.max_velocity:
	    self.set_waypoint_velocity(self.current_waypoints_list, 0, self.max_velocity)
	# set all velocities of other waypoints to 0.0. 
	for i in range(stop_index+1, len(self.current_waypoints_list)):
	    self.set_waypoint_velocity(self.current_waypoints_list, i, 0.0)	

    def get_closest_waypoint_index(self, waypoints_list):
 	'''
	Find the closest waypoint in waypoints_list to vehicle's current position,
	and return its index  
	Note: the return index can be len(waypoints_list). For example, if the car
	has used up all the waypoints_list.
	'''
	if waypoints_list is None:
	    return None

	closest_id = None
	min_dist = 1000000
	for i, waypoint in enumerate(waypoints_list):
	    dist = self.distance_euclid(self.current_pose.position, waypoint.pose.pose.position)
	    if dist < min_dist:
		min_dist = dist
		closest_id = i
	if not self.is_front(closest_id, waypoints_list, self.current_pose):
	    closest_id += 1
	return closest_id 

    def is_front(self, closest_id, waypoints_list, pose2):
	'''
	check whether the found waypoint having closest_id is in the front of the
	vehicle or not.
	'''
	pose1 = waypoints_list[closest_id].pose.pose
	dx = pose1.position.x - pose2.position.x
	dy = pose1.position.y - pose2.position.y
	orientation = pose2.orientation
	_,_,t = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
	lx = math.cos(-t)*dx - math.sin(-t)*dy
	return lx > 0.0



    def distance_euclid(self, p1, p2):
	x, y, z = p1.x-p2.x, p1.y-p2.y, p1.z-p2.z
	return math.sqrt(x**2 + y**2 + z**2)


    def pose_cb(self, msg):
	'''
	 :param msg: of type PoseStamped 
	             (.header, .pose.position, .pose.orientation)
	'''
        self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
        '''
	 :param waypoints: of type Lane
	                   (.header, .waypoints.pose, .waypoints.twist)
	'''
	self.base_waypoints_list = waypoints.waypoints
	self.num_base_waypoints = len(self.base_waypoints_list)
	self.LOOKAHEAD_WPS = 100 if self.num_base_waypoints > 100 else 20


    def traffic_cb(self, msg):
        self.stop_line_index = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
	'''
	 :waypoint: of type styx_msgs/Waypoint (one level lower than Lane)
	'''
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, index, velocity):
	'''
	:param waypoints: of type styx_msgs/Waypoint[]
	:param index: waypoint index in the list waypoints of type Int32
	'''
        waypoints[index].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
	'''
	:param wp1, wp2: indexes, of type Int32
	:param waypoints: of type styx_msgs/Waypoint[]
	This function is used to caculate the distance from waypoint waypoints[wp1]
	to waypoint waypoints[wp2] along the given waypoints.
	'''
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1+1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def callback_current_velocity(self, msg):
        self.current_velocity = msg.twist.linear.x

    #### FOR TEST ####
    def print_velocities(self):
	'''
	test code: print the velocities of self.current_waypoints_list
	'''
	#rospy.logwarn('len(self.current_waypoints_list) = %s', len(self.current_waypoints_list))
	if self.current_waypoints_list is None:
	    return

	velocities = []
	for waypoint in self.current_waypoints_list[:16]:
	    velocities.append(self.get_waypoint_velocity(waypoint))
	rospy.logwarn('       %s', velocities)
    #### TEST END ####


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

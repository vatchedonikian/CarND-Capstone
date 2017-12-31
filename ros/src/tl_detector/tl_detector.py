#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
PUBLISH_RATE = 30
USE_CLASSIFIER = True
USE_SIMULATER = True
#EARLY_WARNING_DISTANCE = 80 # > 11*11/2  2.8*2.8/2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
	
	self.simulator = USE_SIMULATER

	# current pose of the vehicle, composed of position and orientation info
        self.current_pose = None
	# a list of base waypoints
        self.base_waypoints_list = None
	# total number of base waypoints
	self.num_base_waypoints = 0
        self.camera_image = None
	# flag of recieving an image
	self.has_image = False
	#self.has_image = True 
	# a list of lights information. For each of them, it composes of .head, 
	# .pose and .state
        self.lights = []
	# early warning distance, it should be depend on the current velocity
	# In the present version, this is not implemented yet.
	#self.early_warning_distance = EARLY_WARNING_DISTANCE
	self.early_warning_distance = None
	# current velocity with initiate value None
	# it is not used in present version
	self.current_velocity = None
	
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
	# we subscribe to /current_velocity to update the early warning distance
	#sub7 = rospy.Subscriber('/current_velocity', TwistStamped, self.callback_current_velocity)

        config_string = rospy.get_param("/traffic_light_config")
	# change self.config to config
        self.config = yaml.load(config_string)
	# a flag that self.config has add indexes
	self.include_indexes = False
	# for each element  of self.stop_line_positions, the structure is 
	# [x, y, index]. Here, index is the corresponding position of (x,y) 
	# in self.base_waypoints_list.
	self.stop_line_positions = []

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.simulator)
	#APPEARED ONLY ONCE
        self.listener = tf.TransformListener()
	# the current light state
        self.current_state = TrafficLight.UNKNOWN
	# last 'stable' light state
        self.last_state = TrafficLight.UNKNOWN
	# the index of the waypoint of the last time for nearest upcoming red
	# light's stop line. '-1' means the state of the light is green in the
	# last traffic
        self.last_stop_line_index = -1
        self.state_count = 0

	self.lastCrop = None

	rate = rospy.Rate(PUBLISH_RATE)

	while not rospy.is_shutdown():
	    
	    # The premise for publishing traffic light info is image detected
	    if self.has_image:
		stop_line_index, light_state = self.process_traffic_lights()
        	'''
        	Publish upcoming red lights at camera frequency.
        	Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        	of times till we start using it. Otherwise the previous stable
		state is used.
 	        '''
     	        if self.current_state != light_state:
	            self.state_count = 0
	            self.current_state = light_state
	        elif self.state_count >= STATE_COUNT_THRESHOLD:
	            self.last_state = self.current_state
	            stop_line_index = stop_line_index if light_state == TrafficLight.RED or light_state == TrafficLight.YELLOW else -1
	            self.last_stop_line_index = stop_line_index
	            self.upcoming_red_light_pub.publish(Int32(stop_line_index))
		    #rospy.logwarn('1.Publishing traffic_waypoint: %s', Int32(stop_line_index))
	        else:
	            self.upcoming_red_light_pub.publish(Int32(self.last_stop_line_index))
		    #rospy.logwarn('2.Publishing traffic_waypoint: %s', Int32(stop_line_index))
	        self.state_count += 1

	    rate.sleep()

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
	This function uses self.current_pose, self.base_waypoints_list, and
	self.camera_image informations to update the stop_line_index and
	light_state features.

	It will be used after we knew the self.has_image is True. But 
	self.current_pose and self.current_waypoints_list are possibly None. No 
	worry for self.config. It already there when you run this function.

        Returns:
            int: stop_line_index, index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: light_state, ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
	stop_line_index = -1
	light_state = TrafficLight.UNKNOWN

	if self.current_pose is None or self.base_waypoints_list is None:
	    return stop_line_index, light_state
	#if self.stop_line_positions is None:
	#    return stop_line_index, light_state

	##############
	# In the following, self.current_pose and self.base_waypoints_list are
	# not None and We get self.camera_image info
	##############
	# find the vehicle's position
	# Since self.current_pos and self.base_waypoints_list are not None
	# car_position has a meaningful value
	car_position = self.get_closest_waypoint_index(self.current_pose)

        # find the closest stop line index
	stop_index = self.find_closet_stop_index(car_position) 
	if self.distance(self.base_waypoints_list, car_position, stop_index) >=  self.early_warning_distance:
	    return stop_line_index, light_state
	# Otherwise, the stop_index will be set as final stop_line_index
	stop_line_index = stop_index
	# Next, we update the light_state information
	if USE_CLASSIFIER:
	    light_state = self.get_light_state(None)   
	    return stop_line_index, light_state # REVISE LATER
	else:
	    # use the data from the topic /vehicle/traffic_lights self.lights
	    for light in self.lights:
	        p1 = light.pose.pose
		p2 = self.base_waypoints_list[stop_index].pose.pose
		# 40 should be enogh. One can check this by investigating the
		# difference between sim_traffic_light_config.yaml and message from 
		# /vehicle/traffic_lights obtained by the command 'rosmsg echo ...'
	        if self.distance_2D(p1, p2) < 40.:
		    light_state = light.state
		    break
	    return stop_line_index, light_state

        return -1, TrafficLight.UNKNOWN


    def current_pose_cb(self, msg):
	# The self.current_pose contains two parts: .postion and .orientation
        self.current_pose = msg.pose

    def base_waypoints_cb(self, waypoints):
        self.base_waypoints_list = waypoints.waypoints
	self.num_base_waypoints = len(self.base_waypoints_list)
	if self.num_base_waypoints > 200:
	    self.early_warning_distance = 80
	else:
	    self.early_warning_distance = 20

    def traffic_cb(self, msg):
	# self.lights contains list of traffic lights, including pose and state info
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

    #def callback_current_velocity(self, msg):
    #    self.current_velocity = msg

    def get_closest_waypoint_index(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to. Of type geometry_msgs/Pose
	    which is composed of pose.position and pose.orientation

        Returns:
            int: index of the closest waypoint in self.base_waypoints_list. If 
	self.base_waypoints_list is None, return -1
	
	Note:
	    this function doesn't guarantee the returned position is ahead of 'pose'
        """
	closest_id = -1
	if self.base_waypoints_list is None:
	    return closest_id
	
	min_dist = 0
	for index, waypoint in enumerate(self.base_waypoints_list):
	    dist = self.distance_euclid(pose.position, waypoint.pose.pose.position)
	    if closest_id == -1 or dist < min_dist:
		min_dist = dist
		closest_id = index
        return closest_id

    def distance_2D(self, p1, p2):
	'''
	:p1, p2: of type geometry_msgs/Pose 
	:return: 
	'''
	x, y = p1.position.x-p2.position.x, p1.position.y-p2.position.y
	return math.sqrt(x**2+y**2)

    def distance_euclid(self, p1, p2):
	x, y, z = p1.x-p2.x, p1.y-p2.y, p1.z-p2.z
	return math.sqrt(x**2 + y**2 + z**2)

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

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.RED

	# fix camera encoding
	if hasattr(self.camera_image, 'encoding'):
	    if self.camera_image.encoding == '8UC3':
		self.camera_image.encoding = "rgb8"
        else:
            self.camera_image.encoding = 'rgb8'
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        # get classification (simulator classifies whole image)
        if self.simulator:
            return self.light_classifier.get_classification(cv_image)

        # get classification (test site detects boxes using SSD and classifies with
	# OpenCV). if previous location is not known, scan most of the image
        if self.lastCrop is None:
            gostop, found, location = self.light_classifier.get_classification( cv_image[0:500, 50:50+700] )
            if found:
                # check y extents
                if location[0] < 150:
                    top = 0
                    bot = 300
                else:
                    top = location[0]-150
                    if location[0] > 350:
                        top = 200
                        bot = 500
                    else:
                        bot = location[0]+150

                # check x extents (remember, offset by 50)
                if location[1]+50 < 150:
                    left = 50
                    right = 350
                else:
                    left = location[1]+50-150
                    if location[1]+50 > 600:
                        left = 450
                        right = 750
                    else:
                        right = location[1]+50+150
                self.lastCrop = (top,bot, left,right)
                #print "first", self.lastCrop
            # (no else) TL not found, next cycle will be a complete search again
        # otherwise, use last known location as crop starting point
        else:
            (top, bot, left, right) = self.lastCrop
            gostop, found, location = self.light_classifier.get_classification( cv_image[top:bot, left:right] )
            if found: # determine crop for next cycle
                # check y extents, offset by top
                otop, oleft = top, left
                if location[0]+otop < 150:
                    top = 0
                    bot = 300
                else:
                    top = location[0]+otop-150
                    if location[0]+otop > 350:
                        top = 200
                        bot = 500
                    else:
                        bot = location[0]+otop+150
                
                # check x extents, offset by left
                if location[1]+oleft+50 < 200:
                    left = 50
                    right = 350
                else:
                    left = location[1]+oleft+50-150
                    if location[1]+oleft+50 > 600:
                        left = 450
                        right = 750
                    else:
                        right = location[1]+oleft+50+150
                
                self.lastCrop = (top, bot, left, right)
                #print "next", self.lastCrop
            else:
                self.lastCrop = None
                #print "none"
        return gostop

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def add_indexes(self, stop_line_positions):
	'''
	We will add corresponding indexes informations in self.base_waypoints_list
	to stop_line_positions.
	Make sure that self.base_waypoints_list is not None before use.
	:stop_line_positions: stop_line_positions = 
	self.config['stop_line_positions'] from '/vehicle/traffic_lights'
	:return: the element will be of structure [x, y, index]
	'''
	for i in range(len(stop_line_positions)):
	    pose = Pose()
	    pose.position.x = stop_line_positions[i][0]
	    pose.position.y = stop_line_positions[i][1]
	    pose.position.z = 0
	    # the third element of stop_line_positions[i] records the index of stop
	    # line position
	    stop_line_positions[i].append(self.get_closest_waypoint_index(pose))
	return stop_line_positions

    def find_closet_stop_index(self, car_position):
	'''
	This function can only be used when you are sure 1) the car_position has a
	meaningful; 2) self.base_waypoints_list is not None;

	:car_position: int, index of the vehicle in self.base_waypoints_list.
	:return: index of the closest stop waypoint in self.base_waypoints_list. 
	If there is no traffic light and the car is closing the ending line, return
	len(self._base_waypoints_list)-1
	'''
	if not self.include_indexes:
	    # List of positions that correspond to the line to stop in front of 
	    # for a given intersection
	    stop_line_positions = self.config['stop_line_positions']
	    self.stop_line_positions = self.add_indexes(stop_line_positions)
	    self.include_indexes = True
	# initialize closest stop index
	stop_index = self.num_base_waypoints-1
	for i in range(len(self.stop_line_positions)):
	    if car_position < self.stop_line_positions[i][2] < stop_index:
		stop_index = self.stop_line_positions[i][2]
	return stop_index
	    
    def calculate_warning_distance(self, max_a):
	'''
	Assumption: to use this function, make sure self.current_velocity is not
	None. 
 	In this function, we calculate and update the value of early warning
	distance self.early_warning_distance. 

	This function was not used in the present version (process_traffic_lights)
	'''
	if self.current_velocity is None:
	    self.early_warning_distance = EARLY_WARNING_DISTANCE
	else:
	    # from dbw node parameters I found the decelerate is less than 1
	    # S = (vt-v0)**2/(2*a), here vt = 0, a = 1
	    self.early_warning_distance = self.current_velocity*self.current_velocity/2+1

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

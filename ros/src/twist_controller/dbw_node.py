#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
import os
import csv
from twist_controller import Controller
from styx_msgs.msg import Lane, Waypoint


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

DUMP = True #True

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.INFO)
        rospy.logout("dbw_node initiated...........")

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        max_throttle_percentage = rospy.get_param('~max_throttle_percentage', 0.025)
        max_braking_percentage = rospy.get_param('~max_braking_percentage', 0.025)



        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.sampling_rate = 50.0

        params = {
            'vehicle_mass'      : vehicle_mass,
            'fuel_capacity'     : fuel_capacity,
            'brake_deadband'    : brake_deadband,
            'decel_limit'       : decel_limit,
            'accel_limit'       : accel_limit,
            'wheel_radius'      : wheel_radius,
            'wheel_base'        : wheel_base,
            'steer_ratio'       : steer_ratio,
            'max_lat_accel'     : max_lat_accel,
            'max_steer_angle'   : max_steer_angle,
            'sampling_rate'     : self.sampling_rate,
            'max_throttle_percentage' : max_throttle_percentage,
            'max_braking_percentage': max_braking_percentage
        }
        self.controller = Controller(**params)

        self.dbw_enabled = False
        self.current_setpoint = None
        self.current_velocity = None
        self.final_waypoints = None
        self.current_pose = None
        self.dx_c = self.dy_c = self.theta_c = None
        self.steering_feedback = 0.0


        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb,queue_size=1)
        rospy.Subscriber('/vehicle/steering_report', SteeringReport, self.st_report_cb,queue_size=1)

        rospy.loginfo("DBWNode: Publisher and Subscriber initiated")

        self.dbw_data = []

        if DUMP is True:
            base_path = os.path.dirname(os.path.abspath(__file__))
            self.dbwfile = os.path.join(base_path, 'dbw_node.csv')


        self.loop()

    def loop(self):

        rate = rospy.Rate(self.sampling_rate) # Was 50Hz

        while not rospy.is_shutdown():
            if  self.current_velocity is None \
                or self.current_setpoint is None \
                or not self.dbw_enabled:

                #rospy.logwarn('DBWNode.loop: ############Not Initated yet#########')

                continue

            #current_location = self.current_pose.pose.position if self.current_pose.pose.position is not None else None;

            #twist_cmd output required linear and angular velocity
            linear_setpoint = self.current_setpoint.twist.linear.x
            angular_setpoint = self.current_setpoint.twist.angular.z

            #current velocity
            linear_current = self.current_velocity.twist.linear.x
            angular_current = self.current_velocity.twist.angular.z

            throttle, brake, steering = self.controller.control(
                                            linear_setpoint,
                                            angular_setpoint,
                                            linear_current,
                                            self.steering_feedback)

            #if self.dbw_enabled:
            self.publish(throttle, brake, steering)


            if DUMP is True:
                if self.current_pose is not None:
                    self.dx_c = self.current_pose.pose.position.x
                    self.dy_c = self.current_pose.pose.position.y
                    self.theta_c = self.current_pose.pose.position.z

                self.dbw_data.append({
                    'dbw_enabled': self.dbw_enabled,
                    'linear_current': linear_current,
                    'angular_current': angular_current,
                    'linear_required': linear_setpoint,
                    'angular_required': angular_setpoint,
                    'throttle': throttle,
                    'brake': brake,
                    'steer': steering,
                    'dx_c': self.dx_c,
                    'dy_c': self.dy_c,
                    'theta_c': self.theta_c,
                    'steering_feedback': self.steering_feedback})

            rospy.loginfo("DBWNode.loop: throttle, brake, streer"+str(throttle)+' , '+str(brake)+' , '+str(steering))

            rate.sleep()


        #rospy.loginfo('DBWNode.loop dwb_data array length: '+str(len(self.dbw_data)))

        if DUMP is True:
            fieldnames = ['dbw_enabled',
                        'linear_current',
                        'angular_current',
                        'linear_required',
                        'angular_required',
                        'throttle',
                        'brake',
                        'steer',
                        'dx_c',
                        'dy_c',
                        'theta_c',
                        'steering_feedback']

            with open(self.dbwfile, 'w') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.dbw_data)


    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        #rospy.loginfo('TwistController: Steering = ' + str(steer))
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def dbw_enabled_cb(self, msg):
        # check if drive-by-wire is enabled (i.e. the car is not in manual mode)
        self.dbw_enabled = msg.data
        rospy.loginfo('TwistController: dbw_enabled = ' + str(self.dbw_enabled))

        if not self.dbw_enabled:
            self.controller.reset()

    '''
    def current_velocity_cb(self, msg):
        # store the current velocity TwistStamped message
        self.current_vel_linear = msg.twist.linear.x
        rospy.loginfo('DBWNode.current_velocity_cb: current_vel_linear = ' + str(self.current_vel_linear))
    def twist_cmd_cb(self, msg):
        # store the received TwistStamped message from the waypoint follower node
        self.required_vel_linear = msg.twist.linear.x
        self.required_vel_angular = msg.twist.angular.z
        rospy.loginfo('DBWNode.twist_cmd_cb: required_vel_linear = ' + str(self.required_vel_linear))
        rospy.loginfo('DBWNode.twist_cmd_cb: required_vel_angular = ' + str(self.required_vel_angular))
        # debugging veering issue
        if self.required_vel_angular <> self.last_required_vel_angular:
            rospy.loginfo('DBWNode: received new angular velocity required : ' + str(self.required_vel_angular) + ', count = ' + str(self.count_required_vel_angular))
            self.count_required_vel_angular = 0
        self.last_required_vel_angular = self.required_vel_angular
        self.count_required_vel_angular += 1
    '''

    def st_report_cb(self, msg):
        self.steering_feedback = msg.steering_wheel_angle
        #rospy.logwarn("##################STEER FEEDBACK: "+str(self.steering_feedback))

    def twist_cmd_cb(self, msg):
        self.current_setpoint = msg

    def current_velocity_cb(self, msg):
        self.current_velocity = msg

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg

    def current_pose_cb(self, msg):
        self.current_pose = msg

if __name__ == '__main__':
    DBWNode()

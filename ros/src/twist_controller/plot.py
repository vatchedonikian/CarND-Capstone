#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
import os
import csv
from twist_controller import Controller
from styx_msgs.msg import Lane, Waypoint
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from drawnow import *



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

style.use('fivethirtyeight')
fig = plt.figure()
#ax2 = fig.add_axes([-1,1,-1,1])
#plt.subplots(1,1,1)
plt.ion()

class Plot(object):
    def __init__(self):
        rospy.init_node('plot_node', log_level=rospy.INFO)
        rospy.logout("plot_node initiated...........")

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)


        #rospy.loginfo("base_node: Subscriber initiated")

        self.dx=self.dy= self.orientation = 0.0
        self.base_waypoints_list = []
        self.base_waypoints_data = []

        self.dbw_data = []
        plt.show()

        self.loop()



    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if len(self.base_waypoints_list) == 0 and self.dx is None:
                continue;

            for i in range(0, len(self.base_waypoints_list) - 1):
                rospy.loginfo('Plot.dx,dy: '+str(self.base_waypoints_list[i].pose.pose.position.x)+' , '+str(self.base_waypoints_list[i].pose.pose.position.y))
                self.base_waypoints_data.append({
                    'dx': self.base_waypoints_list[i].pose.pose.position.x,
                    'dy': self.base_waypoints_list[i].pose.pose.position.y})

            #rospy.loginfo('Plot.loop base_waypoints_data array length: ' + str(len(self.base_waypoints_data)))


            self.dbw_data.append({
                'dx': self.dx,
                'dy': self.dy,
                'orientation': self.orientation})



            #rospy.loginfo('Plot.loop added to plot')


            self.base_waypoints_list = []
            self.dx = None
            self.dy = None

            drawnow(self.makeFig)
            rate.sleep

    def makeFig(self):

        plt.grid(True)
        x = list([self.base_waypoints_data[i].get("dx")/1000.0 for i in range(len(self.base_waypoints_data))])
        y = list([self.base_waypoints_data[i].get("dy")/1000.0 for i in range(len(self.base_waypoints_data))])
        #ax2.set_xlim()
        plt.plot(x,y,'k')

        x = list([self.dbw_data[i].get("dx")/1000.0 for i in range(len(self.dbw_data))])
        y = list([self.dbw_data[i].get("dy")/1000.0 for i in range(len(self.dbw_data))])
        plt.plot(x, y, 'r<')
        #fig.canvas.draw()

    def waypoints_cb(self, waypoints):
        self.base_waypoints_list = waypoints.waypoints
        #rospy.loginfo('Plot.waypionts_cb: length = ' + str(len(self.base_waypoints_list)))
        #rospy.logwarn('plot.waypionts_cb: length = ' + str(len(self.base_waypoints_list))

    def pose_cb(self, msg):
        # store the current velocity TwistStamped message
        self.dx = msg.pose.position.x
        self.dy = msg.pose.position.y
        self.orientation = msg.pose.orientation
        #rospy.loginfo('Plot.pose_cb: dx,dy,orientation = ' + str(self.dx)+' , '+str(self.dy)+' , '+str(self.orientation))




if __name__ == '__main__':
    Plot()
    #ani = animation.FuncAnimation(fig,p.makeFig(b=p.base_waypoints_data,c=p.dbw_data))
    #plt.show()

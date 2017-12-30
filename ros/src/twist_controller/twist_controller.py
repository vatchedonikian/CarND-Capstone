import rospy                                    # will need access to parameters and system time
from pid import PID                             # PID controller provided
from yaw_controller import YawController        # Yaw controller provided
from lowpass import LowPassFilter
from simplelowpass import SimpleLowPassFilter
import numpy as np
import math

# original Udacity constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_V = 44.704


PID_VEL_P = 1.00
PID_VEL_I = 0.00
PID_VEL_D = 0.00

PID_STR_P = 0.71
PID_STR_I = 0.01
PID_STR_D = 6.7

twiddle_on = False #True #True#True
vel_twiddle_on = False #True

class Controller(object):
    def __init__(self, *args, **kwargs):
        self.vehicle_mass    = kwargs['vehicle_mass']
        self.fuel_capacity   = kwargs['fuel_capacity']
        self.brake_deadband  = kwargs['brake_deadband']
        self.decel_limit 	= kwargs['decel_limit']
        self.accel_limit 	= kwargs['accel_limit']
        self.wheel_radius 	= kwargs['wheel_radius']
        self.wheel_base 		= kwargs['wheel_base']
        self.steer_ratio 	= kwargs['steer_ratio']
        self.max_lat_accel 	= kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.sampling_rate   = kwargs['sampling_rate']
        self.max_braking_percentage= kwargs['max_braking_percentage']
        self.max_throttle_percentage = kwargs['max_throttle_percentage']
        min_speed = 5.0

        #Constant calculated for brake
        self.brake_torque_const = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        #Variable used in control function
        self.delta_t = 1.0 / self.sampling_rate
        self.current_accel = 0.0
        self.linear_past_velocity = 0.0


        #Twiddle parameters for optimizing gains for Velocity
        #Upon observing error on velocity is very low and might
        #not need PID tuning. therefore, only setup Kp=1.0
        self.vel_steps = 100.0
        self.vel_value_error = 0.0
        self.vel_best_error = 9999.00
        self.vel_tolerance = 0.01

        self.vel_p = [PID_VEL_P,PID_VEL_I,PID_VEL_D]
        self.vel_dp = [1.0,1.0,1.0]
        self.vel_i = 0.0
        self.vel_index = None
        self.vel_rule = None
        self.vel_twiddle_started = False
        self.vel_curr_error = 0.0


        #Twiddle parameters for tuning Steering.
        #Tried twiddle parameters and not getting tuned due to
        #local minima
        self.steps = 100.0
        self.value_error = 0.0
        self.best_error = 9999.00
        self.tolerance = 0.02

        self.p = [PID_STR_P,PID_STR_I,PID_STR_D]
        self.dp = [1.0,1.0,1.0]
        self.i = 0.0
        self.index = None
        self.rule = None
        self.twiddle_started = False
        self.curr_error = 0.0


        #yaw controller
        yaw_params = [self.wheel_base, self.steer_ratio, min_speed,
                      self.max_lat_accel,
                      self.max_steer_angle]
        self.yaw_controller = YawController(*yaw_params)

        #Velocity PID
        self.pid_vel_linear = PID(PID_VEL_P, PID_VEL_I, PID_VEL_D,
                                  self.decel_limit, self.accel_limit)

        #restricting between -0.4 and 0.4 i.e. roughly 25 degrees
        self.steer_pid = PID(PID_STR_P, PID_STR_I, PID_STR_D, -0.4363 , 0.4363)#-self.max_steer_angle/2, self.max_steer_angle/2)

        self.tau_steer_correction = 0.2
        self.ts_steer_correction = self.delta_t
        self.low_pass_filter_steer = LowPassFilter(self.tau_steer_correction, self.ts_steer_correction)

        self.last_time = None
        self.last_steering = 0.0

        #twiddle parameters



        rospy.loginfo('TwistController: Complete init')


    def control(self,
                linear_velocity_setpoint,
                angular_velocity_setpoint,
                linear_current_velocity,
                steering_feedback):

        throttle, brake, steering = 0.0, 0.0, 0.0

        time = rospy.get_time()


        if self.last_time is not None:

            dt = time - self.last_time

            self.last_time = time

            # update
            if linear_current_velocity < 1.0:
                self.steer_pid.reset()
                self.pid_vel_linear.reset()

            # use velocity controller compute desired accelaration
            linear_velocity_error =  (linear_velocity_setpoint - linear_current_velocity)/MAX_V
            desired_accel = self.pid_vel_linear.step(linear_velocity_error, dt) #self.delta_t)


            ####################Initiate twiddle######################
            if abs(desired_accel) > 0.0:
                self.vel_twiddle_started = True


            if vel_twiddle_on is True and self.vel_twiddle_started is True:
                #cte value captured and passed on for twiddle.
                #rospy.logwarn('vel cte: ' + str(desired_accel))
                self.vel_twiddle(self.pid_vel_linear.getval_error())

            ########################################################
            if desired_accel > 0.0:
                #if desired_accel < self.accel_limit:
                #    throttle = self.accel_limit
                #else:
                if desired_accel > self.accel_limit:
                    throttle = self.accel_limit
                else:
                    throttle = desired_accel
                brake = 0.0
            else:
                throttle = 0.0
                # reset just to be sure
                #self.accel_pid.reset()
                if abs(desired_accel) > self.brake_deadband:
                    # don't bother braking unless over the deadband level
                    # make sure we do not brake to hard
                    if abs(desired_accel) > abs(self.decel_limit):
                        brake = abs(self.decel_limit) * self.brake_torque_const
                    else:
                        brake = abs(desired_accel) * self.brake_torque_const

            steering = self.yaw_controller.get_steering(linear_velocity_setpoint, angular_velocity_setpoint,
                                                                   linear_current_velocity)



                

            steering = self.steer_pid.step(steering-steering_feedback, dt)
            steering = self.low_pass_filter_steer.filt(steering)

            ####################Initiate twiddle######################

            if abs(steering) > 0.0:
                self.twiddle_started = True

            if twiddle_on is True and self.twiddle_started is True:
                #cte value captured and passed on for twiddle.
                self.twiddle(self.steer_pid.getval_error())

            ##########################################################
            return throttle, brake, steering
        else:
            self.last_time = time
            return 0.0,0.0,0.0

    #Twiddle function implemented as suggested be sebastian.
    #To continue the loop. Once complete one cyle of tuning,
    #best_error reset to 999.0 and tuning will contineu along with
    #PID gains identified by previous cycle.
    #Each cycle is of 200 loops.
    def twiddle(self,cte):
        #if self.best_error > self.tolerance:
        if self.i > self.steps:
            self.curr_error += cte ** 2.0
            #rospy.logwarn("steps: "+str(self.i) + '  '+"curr_error: "+str(self.curr_error))

        if self.i == (self.steps * 2):
            self.curr_error = self.curr_error/(self.steps)

            #reset values
            self.i = 0

            if self.rule is None:
                self.rule = 1

            if self.index is None:
                self.index = 0

            if self.rule == 1:
                self.p[self.index] += self.dp[self.index]
                self.rule = 2

            if self.rule == 2:
                if self.curr_error < self.best_error:
                    self.best_error = self.curr_error
                    self.dp[self.index] *= 1.1
                    self.rule = None
                else:
                    self.p[self.index] -= 2.0 * self.dp[self.index]
                    self.rule = 3 #Only go to rule=3, if error is not better

            if self.rule == 3:
                if self.curr_error < self.best_error:
                    self.best_error = self.curr_error
                    self.dp[self.index] *= 1.1
                else:
                    self.p[self.index] += self.dp[self.index]
                    self.dp[self.index] *= 0.9
                self.rule = None

            # If self.rule = None, then , Index need to be
            # adjust to go for next gain adjustment.
            if self.rule is None:
                if self.index == 2:
                    self.index = 0
                    if self.curr_error > self.best_error * 1.2:
                        self.best_error = 999.00 #self.curr_error
                else:
                    self.index += 1

            self.p[0] = round(self.p[0], 4)
            self.p[1] = round(self.p[1], 4)
            self.p[2] = round(self.p[2], 4)
            rospy.logwarn("index: "+str(self.index)+' rule: '+str(self.rule))
            rospy.logwarn("current err: "+str(self.curr_error)+" best_error: " + str(self.best_error)+' param: '+str(self.p))

            self.steer_pid.set(self.p[0],self.p[1],self.p[2])

            self.curr_error = 0.0

        self.i += 1.0


    #similar like above function. Used for velocity tuning.
    def vel_twiddle(self,cte):
        #if self.best_error > self.tolerance:
        if self.vel_i > self.vel_steps:
            self.vel_curr_error += cte ** 2
            #rospy.logwarn("steps: "+str(self.i) + '  '+"curr_error: "+str(self.curr_error))

        if self.vel_i == (self.vel_steps * 2):
            self.vel_curr_error = self.vel_curr_error/(self.vel_steps)

            #reset values
            self.vel_i = 0

            if self.vel_rule is None:
                self.vel_rule = 1

            if self.vel_index is None:
                self.vel_index = 0

            if self.vel_rule == 1:
                self.vel_p[self.vel_index] += self.vel_dp[self.vel_index]
                self.vel_rule = 2

            if self.vel_rule == 2:
                if self.vel_curr_error < self.vel_best_error:
                    self.vel_best_error = self.vel_curr_error
                    self.vel_dp[self.vel_index] *= 1.1
                    self.vel_rule = None
                else:
                    self.vel_p[self.vel_index] -= 2 * self.vel_dp[self.vel_index]
                    self.vel_rule = 3 #Only go to rule=3, if error is not better

            if self.vel_rule == 3:
                if self.vel_curr_error < self.vel_best_error:
                    self.vel_best_error = self.vel_curr_error
                    self.vel_dp[self.vel_index] *= 1.1
                else:
                    self.vel_p[self.vel_index] += self.vel_dp[self.vel_index]
                    self.vel_dp[self.vel_index] *= 0.9
                self.rule = None

            # If self.rule = None, then , Index need to be
            # adjust to go for next gain adjustment.
            if self.vel_rule is None:
                if self.vel_index == 2:
                    self.vel_index = 0
                    if self.vel_curr_error > self.vel_best_error * 1.2:
                        self.vel_best_error = 999.00 #self.vel_curr_error

                else:
                    self.vel_index += 1

            self.vel_p[0] = round(self.vel_p[0], 4)
            self.vel_p[1] = round(self.vel_p[1], 4)
            self.vel_p[2] = round(self.vel_p[2], 4)
            rospy.logwarn("index: " + str(self.vel_index) + ' rule: ' + str(self.vel_rule))
            rospy.logwarn("vel_twiddle: current err: "+str(self.vel_curr_error)+" best_error: " + str(self.vel_best_error)+' param: '+str(self.vel_p))

            self.pid_vel_linear.set(self.vel_p[0],self.vel_p[1],self.vel_p[2])
            self.vel_curr_error = 0

        self.vel_i += 1.0



    def reset(self):
        self.pid_vel_linear.reset()

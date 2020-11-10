#!/usr/bin/env python
import rospy
import math
import copy
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from twist_filter.cfg import FilterConfig

class TwistFilter(object):
    def __init__(self, components):
        # Set component filters
        self.filters = components

        # Set filter parameters
        self.linear_vel_max = 1
        self.linear_acc_max = 1
        self.angular_vel_max = 1
        self.angular_acc_max = 1
        self.timeout = 0.25
        self.dyn_server = Server(FilterConfig, self.dyn_callback)

        # Set prev values
        self.time_prev = rospy.Time.now()
        self.twist_prev = Twist()

        # Set up publishers/subscribers
        self.sub_cmd_in = rospy.Subscriber('filter_in', Twist, self.update_twist)
        self.pub_cmd_out = rospy.Publisher('filter_out', Twist, queue_size=10)

        ## Uncomment to publish smoothed twist without velocity/acceleration filtering
        # self.pub_cmd_smoothed = rospy.Publisher('filter_smooth', Twist, queue_size=10)

        # Start command publisher
        self.stopped = False
        self.cmd = Twist()
        self.prev_time = rospy.Time.now()
        self.cmd_publisher = rospy.Timer(rospy.Duration(1.0/50.0), self.pub_cmd)

        rospy.loginfo(rospy.get_name() + ': Twist filters ready!')

    def dyn_callback(self, config, level):
        rospy.loginfo("""Filter Reconfigure Request: Linear vel max: {linear_vel_max}, Linear acc max: {linear_acc_max}, \
            Angular vel max:{angular_vel_max}, Angular acc max: {angular_acc_max}, Timeout: {timeout}, \
            Number of samples: {num_samples}, Weights: {weights}, \
            Number of output samples: {num_out_samples}, Output weights: {out_weights}""".format(**config))

        # Component filter update
        self.filters.update_filters(config)

        # Twist filter update
        self.linear_vel_max = config['linear_vel_max']
        self.linear_acc_max = config['linear_acc_max']
        self.angular_vel_max = config['angular_vel_max']
        self.angular_acc_max = config['angular_acc_max']
        self.timeout = config['timeout']
        return config

    def update_twist(self, data):
        self.cmd = data
        self.prev_time = rospy.Time.now()

    def pub_cmd(self, event):
        # Reset twist if we havent gotten an input for specified timeout
        if rospy.Time.now().to_sec() - self.prev_time.to_sec() > self.timeout:
            cmd = self.filter_twist(Twist())
        # Otherwise calculate output twist
        else:
            cmd = self.filter_twist(self.cmd)

        # If a zero twist is calculated, publish only once
        if cmd == Twist():
            if not self.stopped:
                self.pub_cmd_out.publish(cmd)
                self.stopped = True
        # Otherwise keep publishing
        else:
            self.stopped = False
            if cmd != None:
                self.pub_cmd_out.publish(cmd)

    def filter_twist(self, data):
        # Get filtered response and scale to max linear and angular velocities
        cmd_out = Twist()
        for key in self.filters.linear:
            setattr(cmd_out.linear, key, self.filters.linear[key].filter_signal(getattr(data.linear, key)))
        for key in self.filters.angular:
            setattr(cmd_out.angular, key, self.filters.angular[key].filter_signal(getattr(data.angular, key)))

        ## Uncomment to publish smoothed twist without velocity/acceleration filtering
        # self.pub_cmd_smoothed.publish(cmd_out)

        # Get time step
        time_now = rospy.Time.now()
        time_delta = time_now.to_sec() - self.time_prev.to_sec()

        # # Exit if time step is zero
        # if time_delta < 0.0000001:
        #     return

        # Saturate at max velocities and scale
        if self.linear_vel_max > 0 or self.angular_vel_max > 0:
            cmd_out = self._saturate_vel(cmd_out, self.linear_vel_max, self.angular_vel_max)

        # Saturate at max accelerations and scale
        if self.linear_acc_max > 0 or self.angular_acc_max > 0:
            cmd_out = self._saturate_acc(cmd_out, self.linear_acc_max, self.angular_acc_max, time_delta)

        # Update previous values
        self.twist_prev = cmd_out
        self.time_prev = time_now

        return cmd_out

    def _get_twist_mag(self, v):
        '''
        @brief Returns linear and angular magnitudes of a twist

        @param v - Input twist
        @returns lin - Linear magnitude
        @returns ang - Angular magnitude
        '''

        lin = math.sqrt(v.linear.x**2 + v.linear.y**2 + v.linear.z**2)
        ang = math.sqrt(v.angular.x**2 + v.angular.y**2 + v.angular.z**2)

        return lin, ang

    def _get_max_ratios(self, l_mag, a_mag, l_max, a_max):
        '''
        @brief Returns ratio of the vector magnitude to specified max velocity
               and acceleration
            
        @param l_mag - Linear magnitude
        @param a_mag - Angular magnitude
        @param l_max - Linear maximum limit
        @param a_max - Angular maximum limit
        @returns l_r - Linear ratio
        @returns a_r - Angular ratio
        '''

        if l_mag <= l_max:
            l_r = 1.0
        else:
            l_r = l_max / l_mag
        
        if a_mag <= a_max:
            a_r = 1.0
        else:
            a_r = a_max / a_mag

        return l_r, a_r

    def _get_scaling_order(self, ratios):
        '''
        @brief Determines order in which to scale the filtered twist. This is
               used so that the minium amount of scaling is done while still
               following any specified constraints.

        @param ratios - Unsorted array of input ratios
        @returns valid - Sorted array of valid ordered ratios (could be empty)
        '''

        # Select ratios that are less than or equal to 1.0
        valid = []
        for r in ratios:
            if r <= 1.0:
                valid.append(r)
        
        # Sort array from smallest to largest
        if len(valid) > 1:
            valid.sort()

        return valid

    def _saturate_vel(self, v, l_max, a_max):
        sat_twist = copy.deepcopy(v)

        # Saturate linear
        # Get magnitude
        mag = self._get_mag(sat_twist.linear)
        
        # If magnitude is larger than max, saturate
        if mag > l_max:
            # Get ratio of current mag and max mag
            ratio = l_max / mag
            sat_twist.linear.x *= ratio
            sat_twist.linear.y *= ratio
            sat_twist.linear.z *= ratio

        # Saturate Angular
        # Get magnitude
        mag = self._get_mag(sat_twist.angular)
        
        # If magnitude is larger than max, saturate
        if mag > a_max:
            # Get ratio of current mag and max mag
            ratio = a_max / mag
            sat_twist.angular.x *= ratio
            sat_twist.angular.y *= ratio
            sat_twist.angular.z *= ratio
        return sat_twist

    def _get_mag(self, v_comp):
        return math.sqrt(v_comp.x**2 + v_comp.y**2 + v_comp.z**2)

    def _saturate_acc(self, v, l_max, a_max, time_delta):
        sat_twist = copy.deepcopy(v)

        # Get acceleration
        acc = self._get_acc(sat_twist, time_delta)

        # Saturate linear acceleration
        mag = self._get_mag(acc.linear)
        if mag > l_max:
            ratio = l_max / mag
            acc.linear.x *= ratio
            acc.linear.y *= ratio
            acc.linear.z *= ratio

        # Saturate angular acceleration
        mag = self._get_mag(acc.angular)
        if mag > a_max:
            ratio = a_max / mag
            acc.angular.x *= ratio
            acc.angular.y *= ratio
            acc.angular.z *= ratio

        # Calculate saturated twist
        sat_twist.linear.x = (acc.linear.x * time_delta) + self.twist_prev.linear.x
        sat_twist.linear.y = (acc.linear.y * time_delta) + self.twist_prev.linear.y
        sat_twist.linear.z = (acc.linear.z * time_delta) + self.twist_prev.linear.z
        sat_twist.angular.x = (acc.angular.x * time_delta) + self.twist_prev.angular.x
        sat_twist.angular.y = (acc.angular.y * time_delta) + self.twist_prev.angular.y
        sat_twist.angular.z = (acc.angular.z * time_delta) + self.twist_prev.angular.z

        return sat_twist

    # def _saturate_vel(self, v, l_max, a_max):
    #     '''
    #     @brief Saturate and scale input twist according to linear and angular
    #            velocity limits
        
    #     @param v - Input twist
    #     @returns sat_twist - Saturated output twist
    #     '''

    #     sat_twist = Twist()
    #     sat_twist.linear.x = v.linear.x
    #     sat_twist.linear.y = v.linear.y
    #     sat_twist.linear.z = v.linear.z
    #     sat_twist.angular.x = v.angular.x
    #     sat_twist.angular.y = v.angular.y
    #     sat_twist.angular.z = v.angular.z

    #     # Calculate linear and angular magnitudes and get their ratios
    #     l_mag, a_mag = self._get_twist_mag(sat_twist)
    #     l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

    #     # Determine the order in which to scale the twist
    #     scale_order = self._get_scaling_order([l_ratio, a_ratio])
        
    #     # Scale velocities
    #     for r in scale_order:
    #         if l_ratio >= 1.0 and a_ratio >= 1.0:
    #             break
    #         else:
    #             sat_twist = self._scale_twist(sat_twist, r)

    #             # Calculate new linear and angular magnitudes and get their ratios
    #             l_mag, a_mag = self._get_twist_mag(sat_twist)
    #             l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

    #     return sat_twist

    # def _saturate_acc(self, v, l_max, a_max, time_delta):
    #     '''
    #     @brief Scales input twist so that it is constrained by the maximum
    #            linear and angular acceleration limits
        
    #     @param v - Input twist
    #     @param linear_max - Linear acceleration max
    #     @param angular_max - Angular acceleration max
    #     @returns sat_twist - Saturated twist
    #     '''

    #     sat_twist = Twist()
    #     sat_twist.linear.x = v.linear.x
    #     sat_twist.linear.y = v.linear.y
    #     sat_twist.linear.z = v.linear.z
    #     sat_twist.angular.x = v.angular.x
    #     sat_twist.angular.y = v.angular.y
    #     sat_twist.angular.z = v.angular.z

    #     # Get linear acceleration
    #     acc = self._get_acc(sat_twist, time_delta)

    #     # Calculate magnitudes and get their ratios
    #     l_mag, a_mag = self._get_twist_mag(acc)
    #     l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

    #     # Determine the order in which to scale acceleration
    #     scale_order = self._get_scaling_order([l_ratio, a_ratio])

    #     # Scale accelerations
    #     for r in scale_order:
    #         if l_ratio >= 1.0 and a_ratio >= 1.0:
    #             break
    #         else:
    #             # Multiply each acceleration value by the scaling ratio
    #             acc = self._scale_twist(acc, r)

    #             # Calculate new linear and angular magnitudes and get their ratios
    #             l_mag, a_mag = self._get_twist_mag(sat_twist)
    #             l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

    #     # Solve for twist values given the new scaled acceleration values
    #     sat_twist.linear.x = (acc.linear.x * time_delta) + self.twist_prev.linear.x
    #     sat_twist.linear.y = (acc.linear.y * time_delta) + self.twist_prev.linear.y
    #     sat_twist.linear.z = (acc.linear.z * time_delta) + self.twist_prev.linear.z
    #     sat_twist.angular.x = (acc.angular.x * time_delta) + self.twist_prev.angular.x
    #     sat_twist.angular.y = (acc.angular.y * time_delta) + self.twist_prev.angular.y
    #     sat_twist.angular.z = (acc.angular.z * time_delta) + self.twist_prev.angular.z

    #     return sat_twist

    # def _scale_twist(self, v, ratio):
    #     '''
    #     @brief Scales the input twist by the scaling factor

    #     @param v - Input twist
    #     @param ratio - Scaling factor
    #     @returns scaled_twist - Scaled twist
    #     '''

    #     scaled_twist = v
    #     scaled_twist.linear.x *= ratio
    #     scaled_twist.linear.y *= ratio
    #     scaled_twist.linear.z *= ratio
    #     scaled_twist.angular.x *= ratio
    #     scaled_twist.angular.y *= ratio
    #     scaled_twist.angular.z *= ratio

    #     return scaled_twist

    def _get_acc(self, v, time_delta):
        '''
        @brief Returns acceleration of all twist components

        @param v - Current twist
        @param time_delta - Time step
        @returns acc - Twist object that contains acceleration for each component
        '''

        acc = Twist()

        # Calculate acceleration for all twist components
        acc.linear.x = self._get_slope(v.linear.x, self.twist_prev.linear.x, time_delta)
        acc.linear.y = self._get_slope(v.linear.y, self.twist_prev.linear.y, time_delta)
        acc.linear.z = self._get_slope(v.linear.z, self.twist_prev.linear.z, time_delta)
        acc.angular.x = self._get_slope(v.angular.x, self.twist_prev.angular.x, time_delta)
        acc.angular.y = self._get_slope(v.angular.y, self.twist_prev.angular.y, time_delta)
        acc.angular.z = self._get_slope(v.angular.z, self.twist_prev.angular.z, time_delta)

        return acc

    def _get_slope(self, current, prev, step):
        '''
        @brief Returns the acceleration over a given time step

        @param current - Current value
        @param prev - Previous value
        @param step - Time step
        @returns a - Slope (acceleration)
        '''

        a = (current - prev) / step
        return a
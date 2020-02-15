#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from twist_filter.msg import FilterConfig

class TwistFilter(object):
    def __init__(self, components):
        # Load params from parameter server
        try:
            self.linear_vel_max = rospy.get_param('~linear_vel_max')
        except KeyError:
            self.linear_vel_max = 1.0
            rospy.set_param('~linear_vel_max', self.linear_vel_max)
        rospy.loginfo('Setting default max linear velocity to ' + str(self.linear_vel_max))
        
        try:
            self.linear_acc_max = rospy.get_param('~linear_acc_max')
        except KeyError:
            self.linear_acc_max = 1.0
            rospy.set_param('~linear_acc_max', self.linear_acc_max)
        rospy.loginfo('Setting default max linear acceleration to ' + str(self.linear_acc_max))

        try:
            self.angular_vel_max = rospy.get_param('~angular_vel_max')
        except KeyError:
            self.angular_vel_max = 1.0
            rospy.set_param('~angular_vel_max', self.angular_vel_max)
        rospy.loginfo('Setting default max angular velocity to ' + str(self.angular_vel_max))

        try:
            self.angular_acc_max = rospy.get_param('~angular_acc_max')
        except KeyError:
            self.angular_acc_max = 1.0
            rospy.set_param('~angular_acc_max', self.angular_acc_max)
        rospy.loginfo('Setting default max angular acceleration to ' + str(self.angular_acc_max))

        try:
            self.timeout = rospy.get_param('~input_timeout')
        except KeyError:
            self.timeout = 0.25
            rospy.set_param('~input_timeout', self.timeout)
        rospy.loginfo('Setting default input timeout to ' + str(self.timeout) + ' seconds')

        # Set component filters
        self.filters = components

        # Initialize update flag
        self.update_flag = False
        self.update_data = FilterConfig()

        # Set prev values
        self.time_prev = rospy.Time.now()
        self.twist_prev = Twist()

        # Start command publisher
        self.stopped = True
        self.cmd = Twist()
        self.prev_time = rospy.Time.now()
        self.cmd_publisher = rospy.Timer(rospy.Duration(1.0/50.0), self.pub_cmd)

        # Set up publishers/subscribers
        self.sub_config = rospy.Subscriber('filter_config', FilterConfig, self.set_update)
        self.sub_cmd_in = rospy.Subscriber('filter_in', Twist, self.update_twist)
        self.pub_cmd_out = rospy.Publisher('filter_out', Twist, queue_size=10)

        ## Uncomment to publish smoothed twist without velocity/acceleration filtering
        # self.pub_cmd_smoothed = rospy.Publisher('filter_smooth', Twist, queue_size=10)

        rospy.loginfo(rospy.get_name() + ': Twist filters ready!')

    def set_update(self, data):
        '''
        @brief Saves new config data and sets the update_flag to True so the
               config updates can be applied at a safe time.
        
        @param data - Configuration data
        '''

        self.update_flag = True
        self.update_data = data
        rospy.loginfo('Update requested.')

    def update_twist(self, data):
        self.cmd = data
        self.prev_time = rospy.Time.now()

    def update_config(self):
        '''
        @brief Updates linear and angular max values

        @param data - Update message of type FilterConfig
        '''

        # Update only positive nonzero values
        if self.update_data.linear_vel_max > 0:
            self.linear_vel_max = self.update_data.linear_vel_max
            rospy.set_param('~linear_vel_max', self.update_data.linear_vel_max)
        if self.update_data.linear_acc_max > 0:
            self.linear_acc_max = self.update_data.linear_acc_max
            rospy.set_param('~linear_acc_max', self.update_data.linear_acc_max)
        if self.update_data.angular_vel_max > 0:
            self.angular_vel_max = self.update_data.angular_vel_max
            rospy.set_param('~angular_vel_max', self.update_data.angular_vel_max)
        if self.update_data.angular_acc_max > 0:
            self.angular_acc_max = self.update_data.angular_acc_max
            rospy.set_param('~angular_acc_max', self.update_data.angular_acc_max)

        # Reset component filters
        self.filters.linear.x.reset_filter(self.update_data)
        self.filters.linear.y.reset_filter(self.update_data)
        self.filters.linear.z.reset_filter(self.update_data)
        self.filters.angular.x.reset_filter(self.update_data)
        self.filters.angular.y.reset_filter(self.update_data)
        self.filters.angular.z.reset_filter(self.update_data)

        rospy.loginfo('Config values updated!')

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
            self.pub_cmd_out.publish(cmd)

    def filter_twist(self, data):
        # Check if config needs to be updated
        if self.update_flag:
            self.update_config()
            self.update_flag = False

        # Get filtered response and scale to max linear and angular velocities
        cmd_out = Twist()
        cmd_out.linear.x = self.filters.linear.x.filter_signal(data.linear.x)
        cmd_out.linear.y = self.filters.linear.y.filter_signal(data.linear.y)
        cmd_out.linear.z = self.filters.linear.z.filter_signal(data.linear.z)
        cmd_out.angular.x = self.filters.angular.x.filter_signal(data.angular.x)
        cmd_out.angular.y = self.filters.angular.y.filter_signal(data.angular.y)
        cmd_out.angular.z = self.filters.angular.z.filter_signal(data.angular.z)

        ## Uncomment to publish smoothed twist without velocity/acceleration filtering
        # self.pub_cmd_smoothed.publish(cmd_out)

        # Get time step
        time_now = rospy.Time.now()
        time_delta = time_now.to_sec() - self.time_prev.to_sec()

        # Exit if time step is zero
        if time_delta < 0.0000001:
            return

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
        '''
        @brief Saturate and scale input twist according to linear and angular
               velocity limits
        
        @param v - Input twist
        @returns sat_twist - Saturated output twist
        '''

        sat_twist = Twist()
        sat_twist.linear.x = v.linear.x
        sat_twist.linear.y = v.linear.y
        sat_twist.linear.z = v.linear.z
        sat_twist.angular.x = v.angular.x
        sat_twist.angular.y = v.angular.y
        sat_twist.angular.z = v.angular.z

        # Calculate linear and angular magnitudes and get their ratios
        l_mag, a_mag = self._get_twist_mag(sat_twist)
        l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

        # Determine the order in which to scale the twist
        scale_order = self._get_scaling_order([l_ratio, a_ratio])
        
        # Scale velocities
        for r in scale_order:
            if l_ratio >= 1.0 and a_ratio >= 1.0:
                break
            else:
                sat_twist = self._scale_twist(sat_twist, r)

                # Calculate new linear and angular magnitudes and get their ratios
                l_mag, a_mag = self._get_twist_mag(sat_twist)
                l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

        return sat_twist

    def _saturate_acc(self, v, l_max, a_max, time_delta):
        '''
        @brief Scales input twist so that it is constrained by the maximum
               linear and angular acceleration limits
        
        @param v - Input twist
        @param linear_max - Linear acceleration max
        @param angular_max - Angular acceleration max
        @returns sat_twist - Saturated twist
        '''

        sat_twist = Twist()
        sat_twist.linear.x = v.linear.x
        sat_twist.linear.y = v.linear.y
        sat_twist.linear.z = v.linear.z
        sat_twist.angular.x = v.angular.x
        sat_twist.angular.y = v.angular.y
        sat_twist.angular.z = v.angular.z

        # Get linear acceleration
        acc = self._get_acc(sat_twist, time_delta)

        # Calculate magnitudes and get their ratios
        l_mag, a_mag = self._get_twist_mag(acc)
        l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

        # Determine the order in which to scale acceleration
        scale_order = self._get_scaling_order([l_ratio, a_ratio])

        # Scale accelerations
        for r in scale_order:
            if l_ratio >= 1.0 and a_ratio >= 1.0:
                break
            else:
                # Multiply each acceleration value by the scaling ratio
                acc = self._scale_twist(acc, r)

                # Calculate new linear and angular magnitudes and get their ratios
                l_mag, a_mag = self._get_twist_mag(sat_twist)
                l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

        # Solve for twist values given the new scaled acceleration values
        sat_twist.linear.x = (acc.linear.x * time_delta) + self.twist_prev.linear.x
        sat_twist.linear.y = (acc.linear.y * time_delta) + self.twist_prev.linear.y
        sat_twist.linear.z = (acc.linear.z * time_delta) + self.twist_prev.linear.z
        sat_twist.angular.x = (acc.angular.x * time_delta) + self.twist_prev.angular.x
        sat_twist.angular.y = (acc.angular.y * time_delta) + self.twist_prev.angular.y
        sat_twist.angular.z = (acc.angular.z * time_delta) + self.twist_prev.angular.z

        return sat_twist

    def _scale_twist(self, v, ratio):
        '''
        @brief Scales the input twist by the scaling factor

        @param v - Input twist
        @param ratio - Scaling factor
        @returns scaled_twist - Scaled twist
        '''

        scaled_twist = v
        scaled_twist.linear.x *= ratio
        scaled_twist.linear.y *= ratio
        scaled_twist.linear.z *= ratio
        scaled_twist.angular.x *= ratio
        scaled_twist.angular.y *= ratio
        scaled_twist.angular.z *= ratio

        return scaled_twist

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
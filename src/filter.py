#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist

from filter_types import FilterType, MAFilter

class TwistFilterComponent:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None

class TwistFilterObj:
    def __init__(self):
        self.linear = TwistFilterComponent()
        self.angular = TwistFilterComponent()

class TwistFilter:
    def __init__(self):
        # Load params from parameter server
        self.config = rospy.get_param('filter_config')

        # Create filter sample arrays for each twist component
        try:
            self.filters = TwistFilterObj()
            self._set_filter_type()
        except Exception as e:
            rospy.loginfo(e)
            return

        # Set prev values
        self.time_prev = rospy.Time.now()
        self.twist_prev = Twist()

        # Set up publishers/subscribers
        self.sub_cmd_in = rospy.Subscriber('filter_in', Twist, self.filter_twist)
        self.pub_cmd_out = rospy.Publisher('filter_out', Twist, queue_size=10)

        rospy.loginfo('Filters ready!')

    def filter_twist(self, data):
        cmd_out = Twist()

        # Get filtered response
        cmd_out.linear.x = self.filters.linear.x.filter_signal(data.linear.x)
        cmd_out.linear.y = self.filters.linear.y.filter_signal(data.linear.y)
        cmd_out.linear.z = self.filters.linear.z.filter_signal(data.linear.z)
        cmd_out.angular.x = self.filters.angular.x.filter_signal(data.angular.x)
        cmd_out.angular.y = self.filters.angular.y.filter_signal(data.angular.y)
        cmd_out.angular.z = self.filters.angular.z.filter_signal(data.angular.z)

        # Get time step
        time_now = rospy.Time.now()
        time_delta = time_now.to_sec() - self.time_prev.to_sec()

        # Saturate at max accelerations and scale
        if self.config['acc_linear_max'] > 0 or self.config['acc_angular_max'] > 0:
            cmd_out = self._saturate_acc(cmd_out, self.config['acc_linear_max'], self.config['acc_angular_max'], time_delta)

        # Saturate at max velocities and scale
        if self.config['vel_linear_max'] > 0 or self.config['vel_angular_max'] > 0:
            cmd_out = self._saturate_vel(cmd_out, self.config['vel_linear_max'], self.config['vel_angular_max'])

        # Publish output twist
        self.pub_cmd_out.publish(cmd_out)

        # Update previous values
        self.twist_prev = cmd_out
        self.time_prev = time_now

    def _set_filter_type(self):
        # Get filter sample number
        s = self.config['samples']

        # Build desired filters
        if self.config['filter_type'] == FilterType.MAF.value:
            self.filters.linear.x = MAFilter(s)
            self.filters.linear.y = MAFilter(s)
            self.filters.linear.z = MAFilter(s)
            self.filters.angular.x = MAFilter(s)
            self.filters.angular.y = MAFilter(s)
            self.filters.angular.z = MAFilter(s)
        else:
            raise Exception('ERROR - Uknown filter type: ' + str(filter_type))

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

        try:
            l_r = l_max / l_mag
        except ZeroDivisionError:
            l_r = 1.0
        
        try:
            a_r = a_max / a_mag
        except ZeroDivisionError:
            a_r = 1.0

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

        sat_twist = v

        # Calculate linear and angular magnitudes and get their ratios
        l_mag, a_mag = self._get_twist_mag(sat_twist)
        l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

        # Determine the order in which to scale the twist
        scale_order = self._get_scaling_order([l_ratio, a_ratio])
        
        # Scale velocities
        for r in scale_order:
            sat_twist = self._scale_twist(sat_twist, r)

            # Break if new ratios are both >= 1.0 (if scale_order length is larger than 1)
            if len(scale_order) > 1:
                linear_mag, angular_mag = self._get_twist_mag(sat_twist)
                l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

                # No need for more scaling since constraints are already met
                if l_ratio <= 1.0 and a_ratio <= 1.0:
                    break

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

        sat_twist = v

        # Get linear acceleration
        acc = self._get_acc(sat_twist, time_delta)

        # Calculate magnitudes and get their ratios
        l_mag, a_mag = self._get_twist_mag(acc)
        l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

        # Determine the order in which to scale acceleration
        scale_order = self._get_scaling_order([l_ratio, a_ratio])

        # Scale accelerations
        for r in scale_order:
            # Multiply each acceleration value by the scaling ratio
            acc = self._scale_twist(acc, r)

            # Solve for twist values given the new scaled acceleration values
            sat_twist.linear.x = (acc.linear.x * time_delta) + self.twist_prev.linear.x
            sat_twist.linear.y = (acc.linear.y * time_delta) + self.twist_prev.linear.y
            sat_twist.linear.z = (acc.linear.z * time_delta) + self.twist_prev.linear.z
            sat_twist.angular.x = (acc.angular.x * time_delta) + self.twist_prev.angular.x
            sat_twist.angular.y = (acc.angular.y * time_delta) + self.twist_prev.angular.y
            sat_twist.angular.z = (acc.angular.z * time_delta) + self.twist_prev.angular.z

            # Break if new ratios are both >= 1.0 (if scale_order length is larger than 1)
            if len(scale_order) > 1:
                linear_mag, angular_mag = self._get_twist_mag(acc)
                l_ratio, a_ratio = self._get_max_ratios(l_mag, a_mag, l_max, a_max)

                # No need for more scaling since constraints are already met
                if l_ratio <= 1.0 and a_ratio <= 1.0:
                    break

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

        
def main():
    while not rospy.is_shutdown():
        try:
            rospy.init_node('twist_filter')
            f = TwistFilter()
            rospy.spin()
        except rospy.ROSInterruptException as e:
            rospy.loginfo(e)

    rospy.loginfo('Shutting down...')

if __name__ == '__main__':
    main()
#!/usr/bin/env python
import rospy
import math
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
            self.set_filter_type()
        except Exception as e:
            rospy.loginfo(e)
            return

        # Set up publishers/subscribers
        self.sub_cmd_in = rospy.Subscriber('filter_in', Twist, self.filter_twist)
        self.pub_cmd_out = rospy.Publisher('filter_out', Twist, queue_size=10)

        rospy.loginfo('Filters ready!')

    def set_filter_type(self):
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

    def filter_twist(self, data):
        cmd_out = Twist()

        # Get filtered response
        cmd_out.linear.x = self.filters.linear.x.filter_signal(data.linear.x)
        cmd_out.linear.y = self.filters.linear.y.filter_signal(data.linear.y)
        cmd_out.linear.z = self.filters.linear.z.filter_signal(data.linear.z)
        cmd_out.angular.x = self.filters.angular.x.filter_signal(data.angular.x)
        cmd_out.angular.y = self.filters.angular.y.filter_signal(data.angular.y)
        cmd_out.angular.z = self.filters.angular.z.filter_signal(data.angular.z)

        # Saturate at max velocities and scale
        if self.config['vel_linear_max'] > 0:
            cmd_out = self._saturate_twist(cmd_out, self.config['vel_linear_max'], self.config['vel_angular_max'])

        # Saturate at max accelerations and scale

        # Publish output twist
        self.pub_cmd_out.publish(cmd_out)

    def _saturate_twist(self, twist, linear_max, angular_max):
        '''
        @brief Saturate and scale input twist according to linear and angular
               velocity limits
        
        @param twist - Input twist
        @returns sat_twist - Saturated output twist
        '''

        sat_twist = twist

        # Calculate linear and angular magnitudes and get their ratios
        linear_mag = math.sqrt(sat_twist.linear.x**2 + sat_twist.linear.y**2 + sat_twist.linear.z**2)
        angular_mag = math.sqrt(sat_twist.angular.x**2 + sat_twist.angular.y**2 + sat_twist.angular.z**2)
        try:
            linear_ratio = linear_max / linear_mag
        except ZeroDivisionError:
            linear_ratio = 1.0
        
        try:
            angular_ratio = angular_max / angular_mag
        except ZeroDivisionError:
            angular_ratio = 1.0

        # Determine the order in which to scale the twist
        scale_order = []
        if linear_ratio < 1.0 and angular_ratio < 1.0:
            if linear_ratio < angular_ratio:
                scale_order = [linear_ratio, angular_ratio]
            else:
                scale_order = [angular_ratio, linear_ratio]
        else:
            if linear_ratio < 1.0:
                scale_order = [linear_ratio]
            elif angular_ratio < 1.0:
                scale_order = [angular_ratio]
        
        for r in scale_order:
            sat_twist = self._scale_twist(sat_twist, r)

            # Break if new ratios are both >= 1.0
            linear_mag = math.sqrt(sat_twist.linear.x**2 + sat_twist.linear.y**2 + sat_twist.linear.z**2)
            angular_mag = math.sqrt(sat_twist.angular.x**2 + sat_twist.angular.y**2 + sat_twist.angular.z**2)
            try:
                linear_ratio = linear_max / linear_mag
            except ZeroDivisionError:
                linear_ratio = 1.0
            
            try:
                angular_ratio = angular_max / angular_mag
            except ZeroDivisionError:
                angular_ratio = 1.0

            if linear_ratio <= 1.0 and angular_ratio <= 1.0:
                break

        # print [linear_ratio, angular_ratio]
        return sat_twist

    def _scale_twist(self, twist, ratio):
        '''
        @brief Scales the input twist by the scaling factor

        @param twist - Input twist
        @param ratio - Scaling factor
        @returns scaled_twist - Scaled twist
        '''

        scaled_twist = twist
        scaled_twist.linear.x *= ratio
        scaled_twist.linear.y *= ratio
        scaled_twist.linear.z *= ratio
        scaled_twist.angular.x *= ratio
        scaled_twist.angular.y *= ratio
        scaled_twist.angular.z *= ratio

        return scaled_twist
        
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
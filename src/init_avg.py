#!/usr/bin/env python
import rospy
from component_filters import TwistFilterObject, MAFilter
from filter_twist import TwistFilter

def main():
    while not rospy.is_shutdown():
        rospy.init_node('avg_filter')

        # Get size for sample array and check it is a valid size
        num_samples = rospy.get_param('~num_samples')
        if num_samples < 2:
            raise Exception('ERROR - Filter sample size must be larger than 1')
        
        # Create filters for each component of the incoming twist signal
        components = TwistFilterObject()
        components.linear.x = MAFilter()
        components.linear.y = MAFilter()
        components.linear.z = MAFilter()
        components.angular.x = MAFilter()
        components.angular.y = MAFilter()
        components.angular.z = MAFilter()

        # Initialize the twist filter
        twist_filter = TwistFilter(components)

        rospy.spin()

if __name__ == '__main__':
    main()
#!/usr/bin/env python
import rospy
from component_filters import TwistFilterObject, MAFilter
from twist_filter import TwistFilter

def main():
    while not rospy.is_shutdown():
        rospy.init_node('avg_filter')

        # Get size for sample array and check it is a valid size
        samp_size = rospy.get_param('~sample_size')
        if samp_size < 2:
            raise Exception('ERROR - Filter sample size must be larger than 1')
        
        # Create filters for each component of the incoming twist signal
        components = TwistFilterObject()
        components.linear.x = MAFilter(samp_size)
        components.linear.y = MAFilter(samp_size)
        components.linear.z = MAFilter(samp_size)
        components.angular.x = MAFilter(samp_size)
        components.angular.y = MAFilter(samp_size)
        components.angular.z = MAFilter(samp_size)

        # Initialize the twist filter
        twist_filter = TwistFilter(components)

        rospy.spin()

if __name__ == '__main__':
    main()
#!/usr/bin/env python
import rospy
from component_filters import TwistFilterObject, FIRFilter
from filter_twist import TwistFilter

def main():
    while not rospy.is_shutdown():
        rospy.init_node('fir_filter')

        # Get size for sample array and check it is a valid size
        samp_size = rospy.get_param('~sample_size')
        if samp_size < 2:
            raise Exception('ERROR - Filter sample size must be larger than 1')

        # Get list of weights and make sure they equal sample size
        weights = rospy.get_param('~weights')
        if len(weights) != samp_size:
            raise Exception('ERROR - Number of weights must match sample size')
        
        # Create filters for each component of the incoming twist signal
        components = TwistFilterObject()
        components.linear.x = FIRFilter(samp_size, weights)
        components.linear.y = FIRFilter(samp_size, weights)
        components.linear.z = FIRFilter(samp_size, weights)
        components.angular.x = FIRFilter(samp_size, weights)
        components.angular.y = FIRFilter(samp_size, weights)
        components.angular.z = FIRFilter(samp_size, weights)

        # Initialize the twist filter
        twist_filter = TwistFilter(components)

        rospy.spin()

if __name__ == '__main__':
    main()
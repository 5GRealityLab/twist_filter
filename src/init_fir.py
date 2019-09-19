#!/usr/bin/env python
import rospy
from component_filters import TwistFilterObject, FIRFilter
from filter_twist import TwistFilter

def main():
    while not rospy.is_shutdown():
        rospy.init_node('fir_filter')

        # Get size for sample array and check it is a valid size
        num_samples = rospy.get_param('~num_samples')
        if num_samples < 2:
            raise Exception('ERROR - Filter sample size must be larger than 1')

        # Get list of weights and make sure they equal sample size
        weights = rospy.get_param('~weights')
        if len(weights) != num_samples:
            raise Exception('ERROR - Number of weights must match sample size')
        
        # Create filters for each component of the incoming twist signal
        components = TwistFilterObject()
        components.linear.x = FIRFilter()
        components.linear.y = FIRFilter()
        components.linear.z = FIRFilter()
        components.angular.x = FIRFilter()
        components.angular.y = FIRFilter()
        components.angular.z = FIRFilter()

        # Initialize the twist filter
        twist_filter = TwistFilter(components)

        rospy.spin()

if __name__ == '__main__':
    main()
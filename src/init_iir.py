#!/usr/bin/env python
import rospy
from component_filters import TwistFilterObject, IIRFilter
from filter_twist import TwistFilter

def main():
    while not rospy.is_shutdown():
        rospy.init_node('iir_filter')

        # Get size for sample array and check it is a valid size
        in_samp_size = rospy.get_param('~num_samples')
        out_samp_size = rospy.get_param('~num_out_samples')
        if in_samp_size < 1 or out_samp_size < 1:
            raise Exception('ERROR - Filter sample size must be larger than 0')

        # Get list of weights and make sure they equal thier respective sample sizes
        in_weights = rospy.get_param('~weights')
        out_weights = rospy.get_param('~out_weights')
        if len(in_weights) != in_samp_size or len(out_weights) != out_samp_size:
            raise Exception('ERROR - Number of weights must match sample size')
        
        # Create filters for each component of the incoming twist signal
        components = TwistFilterObject()
        components.linear.x = IIRFilter()
        components.linear.y = IIRFilter()
        components.linear.z = IIRFilter()
        components.angular.x = IIRFilter()
        components.angular.y = IIRFilter()
        components.angular.z = IIRFilter()

        # Initialize the twist filter
        twist_filter = TwistFilter(components)

        rospy.spin()

if __name__ == '__main__':
    main()
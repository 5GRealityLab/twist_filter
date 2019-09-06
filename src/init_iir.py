#!/usr/bin/env python
import rospy
from component_filters import TwistFilterObject, IIRFilter
from filter_twist import TwistFilter

def main():
    while not rospy.is_shutdown():
        rospy.init_node('iir_filter')

        # Get size for sample array and check it is a valid size
        in_samp_size = rospy.get_param('~in_sample_size')
        out_samp_size = rospy.get_param('~out_sample_size')
        if in_samp_size < 1 or out_samp_size < 1:
            raise Exception('ERROR - Filter sample size must be larger than 0')

        # Get list of weights and make sure they equal thier respective sample sizes
        in_weights = rospy.get_param('~in_weights')
        out_weights = rospy.get_param('~out_weights')
        if len(in_weights) != in_samp_size or len(out_weights) != out_samp_size:
            raise Exception('ERROR - Number of weights must match sample size')
        
        # Create filters for each component of the incoming twist signal
        components = TwistFilterObject()
        components.linear.x = IIRFilter(in_samp_size, out_samp_size, in_weights, out_weights)
        components.linear.y = IIRFilter(in_samp_size, out_samp_size, in_weights, out_weights)
        components.linear.z = IIRFilter(in_samp_size, out_samp_size, in_weights, out_weights)
        components.angular.x = IIRFilter(in_samp_size, out_samp_size, in_weights, out_weights)
        components.angular.y = IIRFilter(in_samp_size, out_samp_size, in_weights, out_weights)
        components.angular.z = IIRFilter(in_samp_size, out_samp_size, in_weights, out_weights)

        # Initialize the twist filter
        twist_filter = TwistFilter(components)

        rospy.spin()

if __name__ == '__main__':
    main()
#!/usr/bin/env python
from enum import Enum
import rospy

class TwistFilterComponent(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None

class TwistFilterObject(object):
    def __init__(self):
        self.linear = TwistFilterComponent()
        self.angular = TwistFilterComponent()

class FilterType(object):
    def __init__(self):
        # Construct filter sample array
        self.num_samples = rospy.get_param('~num_samples')
        self.samples = [0]*self.num_samples

    def __str__(self):
        '''
        @brief Returns sample array in String form
        '''

        return str(self.samples)

    def update_samples(self, data):
        '''
        @brief Shifts values of entire sample array right 1 space
               and then adds new data element to front

        @param data - New data element
        '''

        if len(self.samples) > 1:
            i = self.num_samples - 2
            while i >= 0:
                self.samples[i+1] = self.samples[i]
                i -= 1

        self.samples[0] = data

    def get_result(self):
        '''
        @brief Returns the filter response. This is a prototype function
               that should be defined in each individual sub-class
        '''

        return 0

    def filter_signal(self, data):
        '''
        @brief Takes in new signal smaple, updates sample array,
               and returns the filtered response

        @param data - New data element
        @returns result
        '''

        self.update_samples(data)
        result = self.get_result()
        return result

    def reset_filter(self, data):
        '''
        @brief Resets filter according to new parameters

        @param data - New configuration
        '''

        if data.num_samples > 0:
            new_samples = [0] * data.num_samples
            if data.num_samples > self.num_samples:
                for s in range(self.num_samples):
                    new_samples[s] = self.samples[s]
            elif data.num_samples < self.num_samples:
                for s in range(data.num_samples):
                    new_samples[s] = self.samples[s]

            self.num_samples = data.num_samples
            self.samples = new_samples

            rospy.set_param('~num_samples', self.num_samples)

class MAFilter(FilterType):
    def __init__(self):
        super(MAFilter, self).__init__()

    def get_result(self):
        '''
        @brief Computes filtered response and returns it

        @returns - result
        '''

        result = 0
        for s in self.samples:
            result += s
        return result / self.num_samples

class FIRFilter(FilterType):
    def __init__(self):
        super(FIRFilter, self).__init__()
        self.weights = rospy.get_param('~weights')

    def get_result(self):
        '''
        @brief Computes filtered response and returns it

        @returns - result
        '''

        result = 0
        for i in range(len(self.samples)):
            result += self.samples[i] * self.weights[i]
        return result

    def reset_filter(self, data):
        '''
        @brief Resets filter according to new parameters

        @param data - New configuration
        '''

        if data.num_samples > 0:
            # Make sure weights match the sample size
            if data.num_samples != len(data.weights):
                rospy.loginfo('Number of weights does not match sample number. Cannot update filter!')
                return

            # Update samples and weights    
            new_samples = [0] * data.num_samples
            if data.num_samples > self.num_samples:
                for s in range(self.num_samples):
                    new_samples[s] = self.samples[s]
            elif data.num_samples < self.num_samples:
                for s in range(data.num_samples):
                    new_samples[s] = self.samples[s]

            self.num_samples = data.num_samples
            self.samples = new_samples
            self.weights = data.weights

            rospy.set_param('~num_samples', self.num_samples)
            rospy.set_param('~weights', self.weights)

class IIRFilter(FilterType):
    def __init__(self):
        super(IIRFilter, self).__init__()
        self.num_out_samples = rospy.get_param('~num_out_samples')
        self.out_samples = [0] * self.num_out_samples
        self.weights = rospy.get_param('~weights')
        self.out_weights = rospy.get_param('~out_weights')

    def update_feedback(self, data):
        '''
        @brief Updates the array of output responses (feedback) with new data point

        @param data - Input feedback
        '''

        if len(self.out_samples) > 1:
            i = self.num_out_samples - 2
            while i >= 0:
                self.out_samples[i+1] = self.out_samples[i]
                i -= 1

        self.out_samples[0] = data

    def get_result(self):
        '''
        @brief Computes filtered response and returns it

        @returns - result
        '''

        result = 0
        input_response = 0
        for i in range(len(self.samples)):
            input_response += self.samples[i] * self.weights[i]

        feedback_response = 0
        for i in range(len(self.out_samples)):
            feedback_response += self.out_samples[i] * self.out_weights[i]

        result = input_response - feedback_response
        return result

    def filter_signal(self, data):
        '''
        @brief Takes in new signal sample, updates sample array,
               and returns the filtered response. It also updates
               feedback sample array after calculating the response

        @param data - New data element
        @returns result
        '''

        self.update_samples(data)
        result = self.get_result()

        # If number is small enough, set it to 0
        if abs(result) < 0.00001:
            result = 0

        self.update_feedback(result)
        return result

    def reset_filter(self, data):
        '''
        @brief Resets filter according to new parameters

        @param data - New configuration
        '''

        # Make sure weights match the sample size
        if data.num_samples != len(data.weights) or data.num_out_samples != len(data.out_weights):
            rospy.loginfo('Number of weights does not match sample number. Cannot update filter!')
            return

        # Update samples and weights
        if data.num_samples > 0: 
            new_samples = [0] * data.num_samples
            if data.num_samples > self.num_samples:
                for s in range(self.num_samples):
                    new_samples[s] = self.samples[s]
            elif data.num_samples < self.num_samples:
                for s in range(data.num_samples):
                    new_samples[s] = self.samples[s]

            self.num_samples = data.num_samples
            self.samples = new_samples
            self.weights = data.weights

            rospy.set_param('~num_samples', self.num_samples)
            rospy.set_param('~weights', self.weights)

        if data.num_out_samples > 0:
            new_samples = [0] * data.num_out_samples
            if data.num_out_samples > self.num_out_samples:
                for s in range(self.num_out_samples):
                    new_samples[s] = self.out_samples[s]
            elif data.num_out_samples < self.num_out_samples:
                for s in range(data.num_out_samples):
                    new_samples[s] = self.out_samples[s]

            self.num_out_samples = data.num_out_samples
            self.out_samples = new_samples
            self.out_weights = data.out_weights

            rospy.set_param('~num_out_samples', self.num_out_samples)
            rospy.set_param('~out_weights', self.out_weights)
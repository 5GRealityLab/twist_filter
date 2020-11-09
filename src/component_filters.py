#!/usr/bin/env python
from enum import Enum
import rospy

class FIRTwistFilterObject(object):
    def __init__(self, config):        
        self.linear = {}
        self.angular = {}

        self.num_samples = 2
        self.weights = []

        # Create filters
        lin_config = config['linear']
        ang_config = config['angular']
        for key in lin_config:
            if lin_config[key]:
                self.linear[key] = FIRFilter(self.num_samples, self.weights)
                rospy.loginfo('Created component filter for linear.' + key)
        for key in ang_config:
            if ang_config[key]:
                self.angular[key] = FIRFilter(self.num_samples, self.weights)
                rospy.loginfo('Created component filter for angular.' + key)

    def update_filters(self, config):
        rospy.loginfo("""Filter Reconfigure Request: Number of samples: {num_samples}, Weights: {weights}""".format(**config))

        weights = []
        if not config['weights'] == '':
            weights = config['weights'].split(',')

        # Only update if number of weights match with respective sample number, or empty weights array defaults to moving average filter
        if config['num_samples'] == len(weights) or len(weights) == 0:
            self.num_samples = config['num_samples']
            self.weights = weights

            # Reset filters
            self.reset_filters(self.num_samples, self.weights)
        else:
            rospy.logwarn('Could not update filter. Make sure number of samples and respective weights match.')

    def reset_filters(self, num_samples, weights):
        for key in self.linear:
            self.linear[key].reset(num_samples, weights)
        for key in self.angular:
            self.angular[key].reset(num_samples, weights)
        rospy.loginfo('Component filters reset.')

class FilterBase(object):
    def __init__(self, num_samples):
        # Construct filter sample array
        self.num_samples = num_samples
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
        @brief Returns the filter response. The default behavior is to
               implement a moving average filter
        '''

        result = 0
        for i in range(len(self.samples)):
            result += self.samples[i]
        return result / len(self.samples)

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

    def reset(self, num_samples, weights):
        '''
        @brief Resets filter according to new parameters

        @param data - New configuration
        '''

        self.num_samples = num_samples
        self.samples = [0] * num_samples

class FIRFilter(FilterBase):
    def __init__(self, num_samples, weights):
        super(FIRFilter, self).__init__(num_samples)
        self.weights = weights

    def get_result(self):
        '''
        @brief Computes filtered response and returns it

        @returns - result
        '''

        result = 0
        if len(self.weights) > 0:
            for i in range(len(self.samples)):
                result += self.samples[i] * self.weights[i]
        else:
            for i in range(len(self.samples)):
                result += self.samples[i]
            result = result / len(self.samples)
        return result

    def reset(self, num_samples, weights):
        self.num_samples = num_samples
        self.samples = [0] * num_samples
        self.weights = weights

class IIRFilter(FilterBase):
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
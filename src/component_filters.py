#!/usr/bin/env python
from enum import Enum

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
    def __init__(self, samples):
        # Construct filter sample array
        self.num_samples = samples
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

        return

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

class MAFilter(FilterType):
    def __init__(self, samples):
        super(MAFilter, self).__init__(samples)

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
    def __init__(self, samples, weights):
        super(FIRFilter, self).__init__(samples)
        self.weights = weights

    def get_result(self):
        '''
        @brief Computes filtered response and returns it

        @returns - result
        '''

        result = 0
        for i in range(len(self.samples)):
            result += self.samples[i] * self.weights[i]
        return result

class IIRFilter(FilterType):
    def __init__(self, in_samples, out_samples, in_weights, out_weights):
        super(IIRFilter, self).__init__(in_samples)
        self.num_out_samples = out_samples
        self.out_samples = [0] * self.num_out_samples
        self.in_weights = in_weights
        self.out_weights = out_weights

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
            input_response += self.samples[i] * self.in_weights[i]

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
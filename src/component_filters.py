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

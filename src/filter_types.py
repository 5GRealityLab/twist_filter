#!/usr/bin/env python
from enum import Enum

class FilterType(Enum):
    MAF = 0
    FIR = 1

class MAFilter:
    def __init__(self, samples):
        # Make sure filter is c=not too small
        if samples < 2:
            raise Exception('ERROR - Fitler sample size must be larger than 1')
            return

        # Construct filter sample array
        self.num_samples = samples
        self.samples = [0]*self.num_samples

    def __str__(self):
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
        @brief Computes filtered response and returns it

        @returns - result
        '''

        result = 0
        for s in self.samples:
            result += s
        return result / self.num_samples

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

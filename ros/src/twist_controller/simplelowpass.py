class SimpleLowPassFilter(object):
    def __init__(self, weight):
        assert .0 <= weight <= 1.
        self.weight = weight
        self.last_value = .0
        self.ready = False

    def filt(self, value):
        if self.ready:
            value = self.weight * value + (1. - self.weight) * self.last_value
            #value = self.weight * value + (1.-self.weight)* self.last_value
        else:
            self.ready = True

        self.last_value = value
        return value

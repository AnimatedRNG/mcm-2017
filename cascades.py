#!/usr/bin/env python3

from collections import namedtuple

LaneParams = namedtuple('LaneParams', 'prob')


class TollElement(object):
    pass


class Lane(TollElement):

    def __init__(self, params):
        self.params = params

    def compute_dist(self):
        return self.params.prob


class Cascade(TollElement):

    def __init__(self, elements, params):
        self.elements = elements
        self.params = params

    def compute_dist(self):
        raise NotImplementedError("Haven't yet implemented this cascade's " +
                                  "compute_dist() function")


class LCascade(Cascade):

    def __init__(self, left, right, params):
        super(LCascade, self).__init__((left, right), params)

    def compute_dist(self):
        alpha = self.elements[0].compute_dist()
        beta = self.elements[1].compute_dist()
        return (beta + (1 - beta) * alpha,)


class RCascade(Cascade):

    def __init__(self, left, right, params):
        super(RCascade, self).__init__((left, right), params)

    def compute_dist(self):
        alpha = self.elements[1].compute_dist()
        beta = self.elements[0].compute_dist()
        return (beta + (1 - beta) * alpha,)


class TriCascade(Cascade):

    def __init__(self, left, middle, right, params):
        super(TriCascade, self).__init__((left, middle, right), params)

    def compute_dist(self):
        alpha = self.elements[0].compute_dist()
        beta = self.elements[1].compute_dist()
        gamma = selfelements[2].compute_dist()
        return (beta +
                (1 - beta) * (1 - alpha) * gamma +
                (1 - beta) * (1 - gamma) * alpha,)


class DivCascade(Cascade):

    def __init__(self, left, middle, right, params):
        super(DivCascade, self).__init__((left, middle, right), params)

    def compute_dist(self):
        alpha = self.elements[0].compute_dist()
        beta = self.elements[1].compute_dist()
        gamma = self.elements[2].compute_dist()
        right = (gamma +
                 (1 - gamma) * alpha * beta +
                 (1 / 2) * (1 - beta) * (1 - alpha) * beta)
        left = (alpha +
                (1 - alpha) * gamma * beta +
                (1 / 2) * (1 - beta) * (1 - gamma) * beta)
        return (left, right)

if __name__ == '__main__':
    a = LCascade(Lane(LaneParams(0.5)), Lane(LaneParams(0.4)), None)
    print(a.compute_dist())

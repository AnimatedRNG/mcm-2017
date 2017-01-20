#!/usr/bin/env python3

from collections import namedtuple

TV = namedtuple('TV', 'h v')
LaneParams = namedtuple('LaneParams', 'h v')
GlobalParams = namedtuple('GlobalParams', 's_max L')

g = GlobalParams(96, 4.8)


class TollElement(object):
    pass


class Lane(TollElement):

    def __init__(self, params):
        self.params = params

    def compute_dist(self):
        return TV(self.params.h, self.params.v)


class Cascade(TollElement):

    def __init__(self, elements, params):
        self.elements = elements
        self.params = params

    def compute_dist(self):
        raise NotImplementedError("Haven't yet implemented this cascade's " +
                                  "compute_dist() function")

    def compute_ratio(self):
        inputs = sum(a.compute_dist().h for a in self.elements)
        outputs = sum(tv.h for tv in self.compute_dist())
        return outputs / inputs


class LCascade(Cascade):

    def __init__(self, left, right, params):
        super(LCascade, self).__init__((left, right), params)

    def compute_dist(self):
        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()

        L = g.L
        s_max = g.s_max

        s_b = b.v / b.h - L
        a_b = min(s_b / s_max, 1)
        return (TV(b.h + a_b * a.h, b.v),)


class RCascade(Cascade):

    def __init__(self, left, right, params):
        super(RCascade, self).__init__((left, right), params)

    def compute_dist(self):
        a = self.elements[1].compute_dist()
        b = self.elements[0].compute_dist()

        L = g.L
        s_max = g.s_max

        s_b = b.v / b.h - L
        a_b = min(s_b / s_max, 1)
        return (TV(b.h + a_b * a.h, b.v),)


class TriCascade(Cascade):

    def __init__(self, left, middle, right, params):
        super(TriCascade, self).__init__((left, middle, right), params)

    def compute_dist(self):
        global g
        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()
        c = self.elements[2].compute_dist()

        L = g.L
        s_max = g.s_max

        # Fix speed
        a_1 = min((1 / s_max) * ((b.v / (b.h + (1 / 2) * a.h)) - L), 1)
        a_2 = min((1 / s_max) * ((b.v / (b.h + (1 / 2) * c.h)) - L), 1)

        return (TV(b.h + a_1 * c.h + a_2 * a.h, b.v), )


class DivCascade(Cascade):

    def __init__(self, left, middle, right, params):
        super(DivCascade, self).__init__((left, middle, right), params)

    def compute_dist(self):
        global g
        H = lambda x: 1 if x > 0 else (1 / 2) if x == 0 else 0
        alpha = lambda t: min((t.v / t.h - L) / s_max, 1)

        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()
        c = self.elements[2].compute_dist()

        L = g.L
        s_max = g.s_max

        # Fix speed
        return (
            TV(a.h + H(alpha(a) - alpha(c)) * alpha(a) * b.h, b.h),
            TV(c.h + H(alpha(c) - alpha(a)) * alpha(c) * b.h, b.h)
        )

if __name__ == '__main__':
    a = DivCascade(
        Lane(LaneParams(0.09, 29)),
        Lane(LaneParams(0.09, 29)),
        Lane(LaneParams(0.09, 29)), None)
    print(a.compute_dist())
    print(a.compute_ratio())

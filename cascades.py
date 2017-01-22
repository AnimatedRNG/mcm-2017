#!/usr/bin/env python3

from collections import namedtuple
from random import random

# $1553 per meter per lane

TV = namedtuple('TV', 'h v')
LaneParams = namedtuple('LaneParams', 'h v')
CascadeParams = namedtuple('CascadeParams', 'm')
GlobalParams = namedtuple('GlobalParams', 's_max L')

g = GlobalParams(96, 4.8)


class TollElement(object):

    def compute_output_num(self):
        raise NotImplementedError("Haven't yet implemented this cascade's " +
                                  "compute_output_num() function")

    def compute_input_num(self):
        raise NotImplementedError("Haven't yet implemented this cascade's " +
                                  "compute_input_num() function")


class Lane(TollElement):

    def __init__(self, params):
        self.params = params

    def compute_dist(self):
        return TV(self.params.h, self.params.v)

    def compute_output_num(self):
        return 1

    def compute_input_num(self):
        return 1

    def compute_length(self):
        return 0

    def __repr__(self):
        return "Lane {{\n\tTV: {}}}".format(self.compute_dist())


class Cascade(TollElement):

    def __init__(self, elements, params):
        self.elements = elements
        self.params = params

    def get_lanes(self):
        return [Lane(LaneParams(tv.h, tv.v)) for tv in self.compute_dist()]

    def compute_length(self):
        raise NotImplementedError("Haven't yet implemented this cascade's " +
                                  "compute_length() function")

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
        return (TV((1 - a_b) * b.h + a_b * a.h, b.v),)

    def compute_length(self):
        return self.elements[0].compute_length() + \
            self.elements[1].compute_length() + \
            self.params.m

    def compute_input_num(self):
        return 2

    def compute_output_num(self):
        return 1

    def __repr__(self):
        return "LCascade {{\n\t{}\n\t{}\n\tTV: {}}}".format(
            self.elements[0], self.elements[1], self.compute_dist())


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
        return (TV((1 - a_b) * b.h + a_b * a.h, b.v),)

    def compute_length(self):
        return self.elements[0].compute_length() + \
            self.elements[1].compute_length() + \
            self.params.m

    def compute_input_num(self):
        return 2

    def compute_output_num(self):
        return 1

    def __repr__(self):
        return "RCascade {{\n\t{}\n\t{}\n\tTV: {}}}".format(
            self.elements[1], self.elements[0], self.compute_dist())


class TriCascade(Cascade):

    def __init__(self, left, middle, right, params):
        super(TriCascade, self).__init__((left, middle, right), params)

    def compute_dist(self):
        global g
        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()
        c = self.elements[2].compute_dist()

        alpha = lambda t: min((t.v / t.h - L) / s_max, 1)

        L = g.L
        s_max = g.s_max

        # Fix speed
        a_1 = min((1 / s_max) * ((b.v / (b.h + (1 / 2) * a.h)) - L), 1)
        a_2 = min((1 / s_max) * ((b.v / (b.h + (1 / 2) * c.h)) - L), 1)

        return (TV(alpha(b) * b.h + a_1 * c.h + a_2 * a.h, b.v), )

    def compute_length(self):
        return self.elements[0].compute_length() + \
            self.elements[1].compute_length() + \
            self.elements[2].compute_length() + \
            self.params.m * 2

    def compute_input_num(self):
        return 3

    def compute_output_num(self):
        return 1

    def __repr__(self):
        return "TriCascade {{\n\t{}\n\t{}\n{}\n\tTV: {}}}".format(
            self.elements[0], self.elements[1], self.elements[2],
            self.compute_dist())


class DivCascade(Cascade):

    def __init__(self, left, middle, right, params):
        super(DivCascade, self).__init__((left, middle, right), params)

    def compute_dist(self):
        global g
        alpha = lambda t: min((t.v / t.h - L) / s_max, 1)

        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()
        c = self.elements[2].compute_dist()

        L = g.L
        s_max = g.s_max

        a_1 = (alpha(a) / (alpha(a) + alpha(c)))
        a_2 = 1 - a_1

        # Fix speed
        return (
            TV(alpha(a) * a.h + a_1 * alpha(a) * b.h, b.v),
            TV(alpha(c) * c.h + a_2 * alpha(c) * b.h, b.v)
        )

    def compute_length(self):
        return self.elements[0].compute_length() + \
            self.elements[1].compute_length() + \
            self.elements[2].compute_length() + \
            self.params.m

    def compute_input_num(self):
        return 3

    def compute_output_num(self):
        return 2

    def __repr__(self):
        return "DivCascade {{\n\t{}\n\t{}\n\t{}\n\tTV: {}}}".format(
            self.elements[0], self.elements[1], self.elements[2],
            self.compute_dist())


def generate(input_lanes, target_number, structures=[]):
    if len(input_lanes) == target_number:
        return [(input_lanes, structures)]
    if len(input_lanes) < target_number:
        return []
    possibilities = []
    for i in range(len(input_lanes) - 1):
        l_cas = LCascade(input_lanes[i], input_lanes[i + 1],
                         CascadeParams(5))
        r_cas = RCascade(input_lanes[i], input_lanes[i + 1],
                         CascadeParams(5))
        possibilities.extend(
            generate(input_lanes[:i] +
                     l_cas.get_lanes() +
                     input_lanes[(i + 2):], target_number, structures + [l_cas]))
        possibilities.extend(
            generate(input_lanes[:i] +
                     r_cas.get_lanes() +
                     input_lanes[(i + 2):], target_number, structures + [r_cas]))
        if (i != len(input_lanes) - 2):
            tri_cas = TriCascade(input_lanes[i],
                                 input_lanes[i + 1],
                                 input_lanes[i + 2], CascadeParams(5))
            div_cas = DivCascade(input_lanes[i],
                                 input_lanes[i + 1],
                                 input_lanes[i + 2], CascadeParams(5))
            possibilities.extend(
                generate(input_lanes[:i] +
                         tri_cas.get_lanes() +
                         input_lanes[(i + 3):],
                         target_number,
                         structures + [tri_cas]))
            possibilities.extend(
                generate(input_lanes[:i] +
                         div_cas.get_lanes() +
                         input_lanes[(i + 3):],
                         target_number,
                         structures + [div_cas]))
    return possibilities

if __name__ == '__main__':
    max_acceptable = 0.1765 * 10
    min_acceptable = 0.036095

    a = [Lane(LaneParams(
        random() * (max_acceptable - min_acceptable) + min_acceptable, 29))
        for a in range(9)]
    best_score = 0
    best = None
    input_h = sum(lane.compute_dist().h for lane in a)
    for config in generate(a, 5):
        output_h = sum(lane.compute_dist().h for lane in config[0])
        score = output_h / input_h
        if best_score < score:
            best_score = score
            best = config
    print(best[1][0].compute_length())
    print("{}:\n{}\n\n".format(best[1], best_score))

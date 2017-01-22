#!/usr/bin/env python3

from collections import namedtuple
from random import random
import sys
# sys.path.append('/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/site-packages')
# import numpy as np

# $1553 per meter per lane

TV = namedtuple('TV', 'h v p')
LaneParams = namedtuple('LaneParams', 'h v p num')
CascadeParams = namedtuple('CascadeParams', 'm')
GlobalParams = namedtuple('GlobalParams', 's_max L')

g = GlobalParams(140, 7.0)  # Used to be 96, 4.8


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
        return TV(self.params.h, self.params.v, self.params.p)

    def compute_output_num(self):
        return 1

    def compute_input_num(self):
        return 1

    def compute_length(self):
        return 0

    def get_name(self):
        return str(self.params.num)

    def __repr__(self):
        return "Lane {{\n\tTV: {}}}".format(self.compute_dist())


class Cascade(TollElement):

    def __init__(self, elements, params):
        for element in elements:
            assert(not isinstance(element, Cascade))
        self.elements = elements
        self.params = params

    def get_lanes(self):
        return [Lane(LaneParams(tv.h, tv.v, tv.p, self.get_output_lane_ids()[i]))
                for i, tv in enumerate(self.compute_dist())]

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

    def get_output_lane_ids(self):
        return [self.elements[1].params.num]

    def compute_dist(self):
        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()

        L = g.L
        s_max = g.s_max

        s_b = b.v / b.h - L
        a_b = min(s_b / s_max, 1)
        return (TV((1 - a_b) * b.h + a_b * a.h, a.v, a.p + b.p - a.p * b.p),)

    def compute_length(self):
        return self.elements[0].compute_length() + \
            self.elements[1].compute_length() + \
            self.params.m

    def compute_input_num(self):
        return 2

    def compute_output_num(self):
        return 1

    def get_name(self):
        return "LC[{}]".format("".join([e.get_name() for e in self.elements]))

    def __repr__(self):
        return "LCascade {{\n\t{}\n\t{}\n\tTV: {}}}".format(
            self.elements[0], self.elements[1], self.compute_dist())


class RCascade(Cascade):

    def __init__(self, left, right, params):
        super(RCascade, self).__init__((left, right), params)

    def get_output_lane_ids(self):
        return [self.elements[0].params.num]

    def compute_dist(self):
        a = self.elements[1].compute_dist()
        b = self.elements[0].compute_dist()

        L = g.L
        s_max = g.s_max

        s_b = b.v / b.h - L
        a_b = min(s_b / s_max, 1)
        return (TV((1 - a_b) * b.h + a_b * a.h, b.v, a.p + b.p - a.p * b.p),)

    def compute_length(self):
        return self.elements[0].compute_length() + \
            self.elements[1].compute_length() + \
            self.params.m

    def compute_input_num(self):
        return 2

    def compute_output_num(self):
        return 1

    def get_name(self):
        return "RC[{}]".format("".join([e.get_name() for e in self.elements]))

    def __repr__(self):
        return "RCascade {{\n\t{}\n\t{}\n\tTV: {}}}".format(
            self.elements[1], self.elements[0], self.compute_dist())


class TriCascade(Cascade):

    def __init__(self, left, middle, right, params):
        super(TriCascade, self).__init__((left, middle, right), params)

    def get_output_lane_ids(self):
        return [self.elements[1].params.num]

    def compute_dist(self):
        global g
        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()
        c = self.elements[2].compute_dist()

        alpha = lambda t: min((t.v / t.h - L) / s_max, 1)

        L = g.L
        s_max = g.s_max

        # Fix speed
        a_1 = min((1. / s_max) * ((b.v / (b.h + (1. / 2.) * a.h)) - L), 1)
        a_2 = min((1. / s_max) * ((b.v / (b.h + (1. / 2.) * c.h)) - L), 1)

        return (TV(alpha(b) * b.h + a_1 * c.h + a_2 * a.h, b.v, b.p + (1 - b.p) * (a.p * (1 - c.p) + c.p * (1 - a.p))), )

    def compute_length(self):
        return self.elements[0].compute_length() + \
            self.elements[1].compute_length() + \
            self.elements[2].compute_length() + \
            self.params.m * 2

    def compute_input_num(self):
        return 3

    def compute_output_num(self):
        return 1

    def get_name(self):
        return "TriC[{}]".format("".join([e.get_name() for e in self.elements]))

    def __repr__(self):
        return "TriCascade {{\n\t{}\n\t{}\n{}\n\tTV: {}}}".format(
            self.elements[0], self.elements[1], self.elements[2],
            self.compute_dist())


class DivCascade(Cascade):

    def __init__(self, left, middle, right, params):
        super(DivCascade, self).__init__((left, middle, right), params)

    def get_output_lane_ids(self):
        return [self.elements[0].params.num, self.elements[2].params.num]

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
            TV(alpha(a) * a.h + a_1 * alpha(a) * b.h, a.v,
               a.p + (1 - a.p) * (1 + c.p) * b.p / 2.0),
            TV(alpha(c) * c.h + a_2 * alpha(c) * b.h, c.v,
               c.p + (1 - c.p) * (1 + a.p) * b.p / 2.0)
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

    def get_name(self):
        return "DivC[{}]".format("".join([e.get_name() for e in self.elements]))

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


def randsol(input_lanes, target_number, structures=[]):
    if len(input_lanes) == target_number:
        return (input_lanes, structures)

    if len(input_lanes) - target_number == 1:
        if target_number != 1:
            rand = int(3 * random())
        else:
            rand = int(2 * random())

        if rand == 0:  # Do LCascade
            index = int(target_number * random())
            l_cas = LCascade(input_lanes[index], input_lanes[index + 1], None)
            input_lanes = input_lanes[:index] + \
                l_cas.get_lanes() + input_lanes[(index + 2):]
            structures.append(l_cas)
        elif rand == 1:  # Do RCascade
            index = int(target_number * random())
            r_cas = RCascade(input_lanes[index], input_lanes[index + 1], None)
            input_lanes = input_lanes[:index] + \
                r_cas.get_lanes() + input_lanes[(index + 2):]
            structures.append(r_cas)
        else:  # Do DivCascade
            index = int((target_number - 1) * random())
            div_cas = DivCascade(input_lanes[index], input_lanes[
                                 index + 1], input_lanes[index + 2], None)
            input_lanes = input_lanes[
                :index] + div_cas.get_lanes() + input_lanes[(index + 3):]
            structures.append(div_cas)
        return (input_lanes, structures)

    else:  # Much room
        rand = int(4 * random())
        if rand == 0:  # Do LCascade
            index = int((len(input_lanes) - 1) * random())
            l_cas = LCascade(input_lanes[index], input_lanes[index + 1], None)
            input_lanes = input_lanes[:index] + \
                l_cas.get_lanes() + input_lanes[(index + 2):]
            structures.append(l_cas)
        elif rand == 1:  # Do RCascade
            index = int((len(input_lanes) - 1) * random())
            r_cas = RCascade(input_lanes[index], input_lanes[index + 1], None)
            input_lanes = input_lanes[:index] + \
                r_cas.get_lanes() + input_lanes[(index + 2):]
            structures.append(r_cas)
        elif rand == 2:  # Do DivCascade
            index = int((len(input_lanes) - 2) * random())
            div_cas = DivCascade(input_lanes[index], input_lanes[
                                 index + 1], input_lanes[index + 2], None)
            input_lanes = input_lanes[
                :index] + div_cas.get_lanes() + input_lanes[(index + 3):]
            structures.append(div_cas)
        else:  # Do TriCascade
            index = int((len(input_lanes) - 2) * random())
            tri_cas = TriCascade(input_lanes[index], input_lanes[
                                 index + 1], input_lanes[index + 2], None)
            input_lanes = input_lanes[
                :index] + tri_cas.get_lanes() + input_lanes[(index + 3):]
            structures.append(tri_cas)
        return randsol(input_lanes, target_number, structures)


def random_ascent(input_lanes, target_number, N=1000000):
    y = randsol(input_lanes, target_number)
    input_h = sum(lane.compute_dist().h for lane in input_lanes)
    input_p = sum(lane.compute_dist().p for lane in input_lanes)
    output_h = sum(lane.compute_dist().h for lane in y[0])
    output_p = sum(lane.compute_dist().p for lane in y[0])
    best_score = output_h / input_h  # Could be p or h
    best = y
    for i in range(N):
        x = randsol(input_lanes, target_number, [])
        output_h = sum(lane.compute_dist().h for lane in x[0])
        output_p = sum(lane.compute_dist().p for lane in x[0])
        score = output_h / input_h
        if score > best_score:
            best_score = score
            best = x
    return (best, best_score)


def num_sols(num_in, num_out):
    # Assumes num_out > 1
    if num_in == num_out:
        return 1
    elif num_in == num_out + 1:
        return 3 * num_out - 1
    else:
        prev = num_sols(num_in - 1, num_out)
        return (3 * num_in - 4) * prev + (num_in - 2) * num_sols(num_in - 2, num_out)


def format_number(solutions):
    answer = str(solutions)
    startIndex = len(answer) % 3
    if startIndex == 0:
        startIndex = 3
    for i in range(startIndex, len(answer) + 4, 4):
        answer = answer[:i] + ',' + answer[i:]
    if answer[-1] == ',':
        answer = answer[:-1]
    return answer


def generate_manual(autonomous, index):
    min_h = .039
    max_h = .111
    std = ((max_h + min_h) / 2.0 - min_h) / 3.0
    # h = min(max((std * np.random.normal() + (min_h + max_h)/2.0), min_h), max_h)
    h = min_h + random() * (max_h - min_h)
    v = 15.6
    p = h * g.L / v
    return Lane(LaneParams(h, v, p, index))


def generate_exact(autonomous, index):
    min_h = .021
    max_h = .139
    std = ((max_h + min_h) / 2.0 - min_h) / 3.0
    # h = min(max((std * np.random.normal() + (min_h + max_h)/2.0), min_h), max_h)
    h = min_h + random() * (max_h - min_h)
    v = 15.6
    p = h * g.L / v
    return Lane(LaneParams(h, v, p, index))


def generate_ezpass(autonomous, index):
    min_h = .2
    max_h = .5  # Assuming EZ pass is half of all lanes out of lack of data
    std = ((max_h + min_h) / 2.0 - min_h) / 3.0
    # h = min(max((std * np.random.normal() + (min_h + max_h)/2.0), min_h), max_h)
    h = min_h + random() * (max_h - min_h)
    v = 20.1  # According to speed limit
    p = h * g.L / v
    return Lane(LaneParams(h, v, p, index))

if __name__ == '__main__':
    max_acceptable = 0.1765 * 10
    min_acceptable = 0.036095
    target_number = 3
    car_length = g.L
    num_manuals = 2
    num_exact = 1
    num_ez_pass = 4
    bests = []
    configs = {}

    for j in range(50):
        a = []
        index = 0
        for i in range(num_manuals):
            l = generate_manual(0, index)
            a.append(l)
            index += 1
        for i in range(num_exact):
            l = generate_exact(0, index)
            a.append(l)
            index += 1
        for i in range(num_ez_pass):
            l = generate_ezpass(0, index)
            a.append(l)
            index += 1

        # best, best_score = random_ascent(a, target_number)
        # print("{}:\n{}\n\n".format(best[1][0:10], best_score))
        # solutions = num_sols(20,5)
        # print(format_number(answer))
        # sys.exit()

        top_n = 3  # Should be small
        best_scores = [0] * top_n
        best = [None] * top_n
        input_h = sum(lane.compute_dist().h for lane in a)
        input_p = sum(lane.compute_dist().p for lane in a)
        possibilities = generate(a, target_number)

        # print("Halfway")
        for config in possibilities:
            output_h = sum(lane.compute_dist().h for lane in config[0])
            output_p = sum(lane.compute_dist().p for lane in config[0])
            score = output_h / input_h  # Could be p or h
            name = ""
            for piece in config[1]:
                name += piece.get_name()
            if j == 0:
                configs[name] = [score]
            else:
                configs[name].append(score)
# if best[-1] == None or best_scores[-1] < score:
# index = top_n - 1
# while ((index > 0) and ((best[index] == None) or (best_scores[index] < score))):
# index -= 1
# if (index == 0) and (best_scores[0] < score):
# index = -1
# index += 1
# best_scores = best_scores[:index] +
# [score] + best_scores[index:-1]
# best = best[:index] + [config] + best[index:-1]
# bests.append(best_scores[0])
##
# for i in range(top_n):
# print("#" + str(i + 1) +
# ": {}:\n{}\n\n".format(best[i][1], best_scores[i]))

    averages = {}
    best_ave = 0
    for config in configs:
        averages[config] = sum(configs[config]) / len(configs[config])
        if averages[config] > best_ave:
            best_ave = averages[config]
            best = config
    print(best, best_ave)
    print(len(possibilities))
    # print(sum(bests) / len(bests))

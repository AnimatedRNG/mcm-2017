#!/usr/bin/env pypy3

from collections import namedtuple
from random import random
from uuid import uuid4
from shutil import rmtree
from os import mkdir, listdir
from os.path import exists, join
import sys
from joblib import Parallel, delayed
# sys.path.append('/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/site-packages')
# import numpy as np

# $1553 per meter per lane

TV = namedtuple('TV', 'h v p')
LaneParams = namedtuple('LaneParams', 'h v p num')
CascadeParams = namedtuple('CascadeParams', 'm')
GlobalParams = namedtuple('GlobalParams', 'L g1 g2')

g = GlobalParams(7.0, 0, 1)  # Used to be 96, 4.8


class TollElement(object):

    def compute_output_num(self):
        raise NotImplementedError("Haven't yet implemented this cascade's " +
                                  "compute_output_num() function")

    def compute_input_num(self):
        raise NotImplementedError("Haven't yet implemented this cascade's " +
                                  "compute_input_num() function")


class Lane(TollElement):

    def __init__(self, params, a=-1):
        self.params = params
        self.L = g.L
        self.ID = a if a != -1 else uuid4()

    def compute_dist(self):
        return TV(self.params.h, self.params.v, self.params.p)

    def get_merging_lane_throughput(self):
        return 0

    def get_merging_lane_probability(self):
        return 0

    def compute_output_num(self):
        return 1

    def compute_input_num(self):
        return 1

    def compute_length(self):
        return 0

    def get_name(self):
        return str(self.params.num)

    def hash(self):
        return hash(this.get_name())

    def __repr__(self):
        return "Lane {{\n\tTV: {}\n\tID: {} }}".format(self.compute_dist(), self.ID)


class Cascade(TollElement):

    def __init__(self, elements, params):
        for element in elements:
            assert(not isinstance(element, Cascade))
        self.elements = elements
        self.params = params

    def hash(self):
        return hash(this.get_name())

    def get_merging_lane_throughput(self):
        output_lanes_ids = [a.params.num for a in self.get_output_lanes()]
        all_lanes = self.elements
        merged_lanes = [
            lane for lane in all_lanes if lane.params.num not in output_lanes_ids]
        return sum(m.params.h for m in merged_lanes)

    def get_merging_lane_probability(self):
        output_lanes_ids = [a.params.num for a in self.get_output_lanes()]
        all_lanes = self.elements
        merged_lanes = [
            lane for lane in all_lanes if lane.params.num not in output_lanes_ids]
        return sum(m.params.p for m in merged_lanes)

    def get_lanes(self):
        input_h = sum(elem.params.h for elem in self.elements)
        output_h = sum(tv.h for tv in self.compute_dist())
        assert(output_h / input_h <= 1 + 1e-6)
        return [Lane(LaneParams(tv.h, tv.v, tv.p,
                                self.get_output_lane_names()[i]),
                     self.get_output_lanes()[i].ID)
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

    def get_output_lanes(self):
        return [self.elements[1]]

    def get_output_lane_names(self):
        return [self.elements[1].params.num]

    def compute_dist(self):
        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()

        L = self.elements[0].L
        s_max = L * 20

        s_b = b.v / b.h - L
        a_b = min(s_b / s_max, 1)
        return (TV(b.h + a_b * a.h,
                   b.v,
                   a.p + b.p - a.p * b.p),)

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

    def get_output_lanes(self):
        return [self.elements[0]]

    def get_output_lane_names(self):
        return [self.elements[0].params.num]

    def compute_dist(self):
        a = self.elements[1].compute_dist()
        b = self.elements[0].compute_dist()

        L = self.elements[1].L
        s_max = L * 20

        s_b = b.v / b.h - L
        a_b = min(s_b / s_max, 1)
        return (TV(b.h + a_b * a.h,
                   b.v,
                   a.p + b.p - a.p * b.p),)

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

    def get_output_lanes(self):
        return [self.elements[1]]

    def get_output_lane_names(self):
        return [self.elements[1].params.num]

    def compute_dist(self):
        global g
        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()
        c = self.elements[2].compute_dist()

        alpha = lambda t, L, s_max: min((t.v / t.h - L) / s_max, 1)

        L_a = self.elements[0].L
        L_b = self.elements[1].L
        L_c = self.elements[2].L

        s_max_a = 20 * L_a
        s_max_b = 20 * L_b
        s_max_c = 20 * L_c

        # Fix speed
        a_1 = min((1. / s_max_b) *
                  (((b.v + a.v / 2.) / (b.h + a.h / 2.)) -
                   (L_b + L_a / 2.)), 1)
        a_2 = min((1. / s_max_b) *
                  (((b.v + c.v / 2.) / (b.h + c.h / 2.)) -
                   (L_b + L_c / 2.)), 1)

        return (TV(b.h + a_1 * c.h + a_2 * a.h,
                   b.v,
                   b.p + (1 - b.p) * (a.p + c.p - 1.5 * a.p * c.p)),)

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

    def get_output_lanes(self):
        return [self.elements[0], self.elements[2]]

    def get_output_lane_names(self):
        return [self.elements[0].params.num, self.elements[2].params.num]

    def compute_dist(self):
        global g
        alpha = lambda t, L, s_max: min((t.v / t.h - L) / s_max, 1)

        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()
        c = self.elements[2].compute_dist()

        L_a = self.elements[0].L
        L_b = self.elements[1].L
        L_c = self.elements[2].L

        s_max_a = L_a * 20
        s_max_b = L_b * 20
        s_max_c = L_c * 20

        a_1 = (alpha(a, L_a, s_max_a) / (alpha(a, L_a, s_max_a) +
                                         alpha(c, L_c, s_max_c)))
        a_2 = 1 - a_1

        # Fix speed
        return (
            TV(a.h + a_1 * alpha(a, L_a, s_max_a) *
               b.h, a.v,
               a.p + (1 - a.p) * (1 + c.p) * b.p / 2.0),
            TV(c.h + a_2 * alpha(c, L_c, s_max_c) *
               b.h, c.v,
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
        with_singletons = structures[:]
        output_lanes_ids = set()
        for struct in structures:
            output_lanes = struct.get_output_lanes()
            for output_lane in output_lanes:
                output_lanes_ids.add(output_lane.ID)
        for lane in input_lanes:
            lane_id = lane.ID
            if not lane_id in output_lanes_ids:
                with_singletons.append(lane)
        return [(input_lanes, with_singletons)]
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
    # h = min(max((std * np.random.normal() + (min_h + max_h)/2.0), min_h),
    # max_h)
    h = min_h + random() * (max_h - min_h)
    v = 15.6
    p = h * g.L / v
    return Lane(LaneParams(h, v, p, index))


def generate_exact(autonomous, index):
    min_h = .021
    max_h = .139
    std = ((max_h + min_h) / 2.0 - min_h) / 3.0
    # h = min(max((std * np.random.normal() + (min_h + max_h)/2.0), min_h),
    # max_h)
    h = min_h + random() * (max_h - min_h)
    v = 15.6
    p = h * g.L / v
    return Lane(LaneParams(h, v, p, index))


def generate_ezpass(autonomous, index):
    min_h = .2
    max_h = .5  # Assuming EZ pass is half of all lanes out of lack of data
    std = ((max_h + min_h) / 2.0 - min_h) / 3.0
    # h = min(max((std * np.random.normal() + (min_h + max_h)/2.0), min_h),
    # max_h)
    h = min_h + random() * (max_h - min_h)
    v = 20.1  # According to speed limit
    p = h * g.L / v
    return Lane(LaneParams(h, v, p, index))


def generate_lane_perms(curr_string, target_length):
    if len(curr_string) == target_length:
        return [curr_string]
    else:
        possibilities = []
        possibilities.extend(
            generate_lane_perms(curr_string + 'a', target_length))
        possibilities.extend(
            generate_lane_perms(curr_string + 'b', target_length))
        possibilities.extend(
            generate_lane_perms(curr_string + 'c', target_length))
        return possibilities


def find_optimal(lane_ordering, num_trials=100):
    target_number = 2
    bests = []
    configs = {}

    for j in range(num_trials):
        a = []
        index = 0
        for i in lane_ordering:
            if i == 'a':
                l = generate_manual(0, index)
            elif i == 'b':
                l = generate_exact(0, index)
            elif i == 'c':
                l = generate_ezpass(0, index)
            else:
                assert false
            a.append(l)
            index += 1

        top_n = 3  # Should be small
        best_h_scores = [0] * top_n
        best_p_scores = [0] * top_n
        best_hs = [None] * top_n
        best_ps = [None] * top_n
        input_h = sum(lane.compute_dist().h for lane in a)
        input_p = sum(lane.compute_dist().p for lane in a)
        possibilities = generate(a, target_number)

        for config in possibilities:
            output_h = sum(lane.compute_dist().h for lane in config[0])
            output_p = sum(lane.compute_dist().p for lane in config[0])
            throughput_score = output_h / input_h  # Could be p or h
            probability_score = output_p / input_p
            assert(throughput_score < 1 + 1e6)
            assert(probability_score < 1 + 1e6)
            if throughput_score > 1 + 1e6:
                print("Throughput {} is greater than 1!"
                      .format(throughput_score))
                print("Input lanes: \n{}".format(a))
                print("Output lanes: \n{}".format(config[0]))
                print("Name: {}\n".format(
                    "".join(piece.get_name() for piece in config[1])))
            if probability_score > 1 + 1e6:
                print("Probability {} is greater than 1!"
                      .format(probability_score))
                print("Input lanes: \n{}".format(a))
                print("Output lanes: \n{}".format(config[0]))
                print("Name: {}\n".format(
                    "".join(piece.get_name() for piece in config[1])))

            b_l = len(lane_ordering) - target_number
            merging_lane_h = sum(struct.get_merging_lane_throughput()
                                 for struct in config[1])
            merging_lane_p = sum(struct.get_merging_lane_probability()
                                 for struct in config[1])
            score = g.g1 * merging_lane_h / (0.059683 * b_l) + \
                g.g2 * throughput_score
            name = ""
            for piece in config[1]:
                name += piece.get_name()
            if j == 0:
                configs[name] = [[score], [probability_score], str(config)]
            else:
                configs[name][0].append(score)
                configs[name][1].append(probability_score)

    averages = {}
    best_ave = 0
    p_averages = {}
    best_p_ave = 0
    best_config_str = None
    best_p_config_str = None
    for config in configs:
        averages[config] = sum(configs[config][0]) / len(configs[config][0])
        p_averages[config] = sum(configs[config][1]) / len(configs[config][1])
        if averages[config] > best_ave:
            best_ave = averages[config]
            best_config_str = str(configs[config][2])
            best = config
        if p_averages[config] > best_p_ave:
            best_p_ave = p_averages[config]
            best_p_config_str = str(configs[config][2])
            best_p = config
    with open(join("tmp", str(uuid4())), "w+") as fp:
        fp.write("{}\n".format(lane_ordering))
        fp.write("{}\n".format(best))
        fp.write("{}\n".format(best_ave))
        fp.write("{}\n".format(best_config_str))
    with open(join("tmpps", str(uuid4())), "w+") as fp:
        fp.write("{}\n".format(lane_ordering))
        fp.write("{}\n".format(best_p))
        fp.write("{}\n".format(best_p_ave))
        fp.write("{}\n".format(best_p_config_str))
    print(best_ave)
    print(best_config_str)
    print()
    print(best_p_ave)
    print(best_p_config_str)
    print(len(possibilities))
    # print(sum(bests) / len(bests))


if __name__ == '__main__':
    if exists("tmp"):
        rmtree("tmp")
    mkdir("tmp")
    if exists("tmpps"):
        rmtree("tmpps")
    mkdir('tmpps')
    lp = [l for l in generate_lane_perms("", 6)
          if l.count('a') > 0 and
          (l.count('b') == 1 or l.count('b') == 2) and
          (l.count('c') > 0 and l.count('c') < 4)]
    Parallel(n_jobs=31)(delayed(find_optimal)(l) for l in lp)

    all_configs = {}
    for log in listdir('tmp'):
        exact_path = join('tmp', log)
        with open(exact_path, "r") as fp:
            ordering = fp.readline().rstrip()
            name = fp.readline().rstrip()
            average = float(fp.readline().rstrip())
            cf = "".join(a for a in fp.readlines())
            if name in all_configs:
                print("Updating")
                all_configs[name][0] += average
                all_configs[name][1] += 1
            else:
                all_configs[name] = [average, 1, cf]
    all_p_configs = {}
    for log in listdir('tmpps'):
        exact_path = join('tmpps', log)
        with open(exact_path, "r") as fp:
            ordering = fp.readline().rstrip()
            name = fp.readline().rstrip()
            p_average = float(fp.readline().rstrip())
            cf = "".join(a for a in fp.readlines())
            if name in all_p_configs:
                print("Updating P's")
                all_p_configs[name][0] += p_average
                all_p_configs[name][1] += 1
            else:
                all_p_configs[name] = [p_average, 1, cf]

    best_average = -1
    best_value = None
    best_p_average = -1
    best_p_value = None
    for name, average_cf in all_configs.items():
        average = average_cf[0] / average_cf[1] \
            if average_cf[1] != 0 else average_cf[0]
        if average > best_average:
            best_average = average
            best_value = average_cf[2]
    for name, average_cf in all_p_configs.items():
        average = average_cf[0] / average_cf[1] \
            if average_cf[1] != 0 else average_cf[0]
        if average > best_p_average:
            best_p_average = average
            best_p_value = average_cf[2]
    print("Best average: {}".format(best_average))
    print("Best value:\n{}".format(best_value))
    print()
    print("Best probabilty average: {}".format(best_p_average))
    print("Best probability value:\n{}".format(best_p_value))

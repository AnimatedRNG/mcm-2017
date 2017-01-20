#!/usr/bin/env python3

from collections import namedtuple

TV = namedtuple('TV', 'h v')
LaneParams = namedtuple('LaneParams', 'h v')
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

    def __repr__(self):
        return "Lane {{\n\tTV: {}}}".format(self.compute_dist())


class Cascade(TollElement):

    def __init__(self, elements, params):
        self.elements = elements
        self.params = params

    def get_lanes(self):
        return [Lane(LaneParams(tv.h, tv.v)) for tv in self.compute_dist()]

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
        return (TV(b.h + a_b * a.h, b.v),)

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

        L = g.L
        s_max = g.s_max

        # Fix speed
        a_1 = min((1 / s_max) * ((b.v / (b.h + (1 / 2) * a.h)) - L), 1)
        a_2 = min((1 / s_max) * ((b.v / (b.h + (1 / 2) * c.h)) - L), 1)

        return (TV(b.h + a_1 * c.h + a_2 * a.h, b.v), )

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
        H = lambda x: 1 if x > 0 else (1 / 2) if x == 0 else 0
        alpha = lambda t: min((t.v / t.h - L) / s_max, 1)

        a = self.elements[0].compute_dist()
        b = self.elements[1].compute_dist()
        c = self.elements[2].compute_dist()

        L = g.L
        s_max = g.s_max

        # Fix speed
        return (
            TV(a.h + H(alpha(a) - alpha(c)) * alpha(a) * b.h, b.v),
            TV(c.h + H(alpha(c) - alpha(a)) * alpha(c) * b.h, b.v)
        )

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
        l_cas = LCascade(input_lanes[i], input_lanes[i + 1], None)
        r_cas = RCascade(input_lanes[i], input_lanes[i + 1], None)
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
                                 input_lanes[i + 2], None)
            div_cas = DivCascade(input_lanes[i],
                                 input_lanes[i + 1],
                                 input_lanes[i + 2], None)
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
    a = [Lane(LaneParams(0.2, 29)) for a in range(8)]
    best_score = 0
    best = None
    input_h = sum(lane.compute_dist().h for lane in a)
    for config in generate(a, 3):
        output_h = sum(lane.compute_dist().h for lane in config[0])
        score = output_h / input_h
        if best_score < score:
            best_score = score
            best = config
    print("{}:\n{}\n\n".format(best[1], best_score))

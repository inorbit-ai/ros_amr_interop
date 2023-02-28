#!/usr/bin/env python3
# Copyright 2023 InOrbit, Inc.

"""Utility functions"""

import math


def angle_diff(phi, theta):
    """ Angle from theta to phi """
    return (phi - theta + math.pi) % (2 * math.pi) - math.pi


def euc_distance(x1, y1, x2, y2):
    """ Euclidean distance between point 1 and 2 """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

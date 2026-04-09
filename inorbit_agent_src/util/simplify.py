# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# The work in this file is based on Simplify.js, a high-performance JS
# polyline simplification library:
# https://github.com/mourner/simplify-js
# Parts of the code were taken and modified. The original license
# is reproduced below.
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Vladimir Agafonkin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from .math_util import point_sq_distance
from .math_util import point_to_line_sq_distance


def simplify(points, tolerance=1, highest_quality=False):
    """
    Simplifies the input points array using radial distance (if highest
    quality is not required) and Douglas-Peuquer algorithm.
    """

    if len(points) <= 2:
        return points

    if tolerance is None:
        tolerance = 1

    if not highest_quality:
        points = simplify_radial_dist(points, tolerance)

    points = simplify_DouglasPeucker(points, tolerance)

    return points


def simplify_radial_dist(points, sq_tolerance):
    """
    Basic distance-based simplification.
    """

    prev_point = points[0]
    new_points = [prev_point]

    for point in points:
        if point_sq_distance(point, prev_point) > sq_tolerance:
            new_points.append(point)
            prev_point = point

    # Make sure the ending point is stored
    if prev_point != point:
        new_points.append(point)

    return new_points


def simplify_DouglasPeucker(points, sq_tolerance):
    """
    Simplification using Ramer-Douglas-Peucker algorithm.
    """

    last = len(points) - 1

    simplified = [points[0]]
    simplify_DP_step(points, 0, last, sq_tolerance, simplified)
    simplified.append(points[last])

    return simplified


def simplify_DP_step(points, first, last, sq_tolerance, simplified):
    """
    Does the core calculation of the Ramer-Douglas-Peucker algorithm and
    decides which points should be preserved from the original array.
    """

    max_sq_dist = sq_tolerance

    for i in range(first + 1, last):
        sq_dist = point_to_line_sq_distance(points[i], points[first], points[last])

        if sq_dist > max_sq_dist:
            index = i
            max_sq_dist = sq_dist

    if max_sq_dist > sq_tolerance:
        if index - first > 1:
            simplify_DP_step(points, first, index, sq_tolerance, simplified)
        simplified.append(points[index])
        if last - index > 1:
            simplify_DP_step(points, index, last, sq_tolerance, simplified)

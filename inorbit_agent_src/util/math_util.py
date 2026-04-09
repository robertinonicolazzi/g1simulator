# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Useful functions to deal with
#  - 2D and 3D point operations (distance, squared distance, distance from point to segment).
#  - Paths / polylines (distance between paths)
#  - Arrays: downsampling to N elements, at regular intervals.
import math

import numpy as np


def point_distance(p1, p2):
    return math.sqrt(point_sq_distance(p1, p2))


def point_sq_distance(p1, p2):
    """
    Square distance between 2 points
    """

    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = getattr(p1, "z", 0) - getattr(p2, "z", 0)

    return dx * dx + dy * dy + dz * dz


def point_to_line_sq_distance(p, p1, p2):
    """
    2D square distance from a point p to a line given by p1-p2
    """

    x = p1.x
    y = p1.y
    dx = p2.x - x
    dy = p2.y - y

    if dx != 0 or dy != 0:
        t = ((p.x - x) * dx + (p.y - y) * dy) / (dx * dx + dy * dy)

        if t > 1:
            x = p2.x
            y = p2.y
        elif t > 0:
            x += dx * t
            y += dy * t

    dx = p.x - x
    dy = p.y - y

    return dx * dx + dy * dy


def path_distance(path1, path2):
    """
    Distance between two paths.
    - Returns inf if any of the paths are invalid,
    they have different size or they differ in more than 2 points.
    - Returns the number of points the paths differ when this is less than
    2.

    NOTE: For paths larger than 1000 points, this method doesn't compare
    paths over their whole length but just on a 20% of them.
    """

    inf = float("inf")

    if path1 is None and path2 is None:
        return 0.0  # Both are None
    if (path1 is None) != (path2 is None):  # (Note: Logical XOR!)
        return inf
    if len(path1) != len(path2):  # Not same number of points
        return inf

    differ = 0  # Number of points they differ
    dist = 0  # Total distance

    # PATHS LONGER THAN 1000 POINTS
    # Avoid comparing point to point when the path has more than 1000
    # points. Instead, compute how many points the 20% of the path's length
    # represent, divide that in 3 (m = 6.6% of total points) and then:
    #   - Compare the first m points
    #   - Compare the mid m points
    #   - Compare the last m points

    if len(path1) > 1000:
        # Just compare over the 20% of the path's size, divided in 3 times
        check_step = int(len(path1) * 0.2 / 3)  # m

        initial_batch = np.arange(check_step)
        mid_batch = np.arange(
            int(len(path1) * 0.5 - check_step / 2), int(len(path1) * 0.5 + check_step / 2)
        )
        final_batch = np.arange(len(path1) - check_step, len(path1))

        # Concatenate all the indexes of path's data that needs to be
        # compared.
        indexes = np.concatenate((initial_batch, mid_batch, final_batch), axis=0)
    else:
        # PATHS SHORTER THAN 500 POINTS
        indexes = range(len(path1))

    # Compare points
    for i in range(len(path1)):
        p1 = path1[i]
        p2 = path2[i]
        d = point_distance(p1, p2)
        dist += d
        if d > 0:
            differ += 1
    if differ > 1:
        # More than two points differ. We rather resend the path.
        return inf
    else:
        # Return 0 (if paths are equal) or the distance in the only point
        # they differ.
        return dist


def downsample_array(arr, maxn):
    """
    Downsamples (any) array to N elements, taking at regular
    intervals. First and last element are always returned (provided N>=2).
    """

    if len(arr) <= maxn or maxn <= 1:
        # It is assumed that _downsample will return a new array,
        # so just save the iterators logic but still return a copy
        return arr[:]

    # Take exactly maxn elements. Select at regular (non-int) intervals
    # maxn-1 of them, and the last one manually.
    return [arr[int(float(i * len(arr)) / (maxn - 1))] for i in range(maxn - 1)] + [arr[-1]]


def get_position_with_offset(pos_x, pos_y):
    """
    Calculates offsets for poses with large coordinates.

    In some occasions pose coordinates are likely very large numbers which may cause losing
    precision when transmitting them as protobuf messages. To avoid this, coordinates are
    separated in a high-order offset component and the low-order higher accuracy part for
    using them on Pose-related messages e.g. PoseMessageData and LocationAndPoseMessage.

    Example:
    get_position_with_offset(173285.4100000000035, 1328333.1799999999348)

    returns:
    {
        "pos_x": 3285.4100000000035,
        "pos_y": -1666.8200000000652,
        "offset_x": 170000,
        "offset_y": 1330000
    }
    """

    POSITION_MAX_VALUE = 10000 # Position will be in interval (-10000.0, 10000.0)
    
    offset_x = round(pos_x / POSITION_MAX_VALUE) * POSITION_MAX_VALUE
    pos_x = pos_x - offset_x
    offset_y = round(pos_y / POSITION_MAX_VALUE) * POSITION_MAX_VALUE
    pos_y = pos_y - offset_y
    return { 'pos_x': pos_x, 'pos_y': pos_y, 'offset_x': offset_x, 'offset_y': offset_y }

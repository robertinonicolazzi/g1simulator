# Copyright (c) 2022, InOrbit, Inc.
# All rights reserved.
# Array utility functions, including our own encodings and compression of arrays.
# TODO(herchu) Move the encoding used for laser ranges from
# LocalizationAgentlet#_encode_floating_point_list to this file.
# (This encoding is based on runs of equal values, unlike the DeltaInt encoding below.)
"""
The delta_int_* functions below implement our "DeltaInt" encoding:
Used to encode arrays of floating point numbers which are relatively close to each other
in a more integer-friendly form, with minimal precision loss.

This encoding allows serializing values in protobuf in a much more compact precision
than floats, as small integers (protobuf: signed ints, `sint32`) use fewer bits
(proportional to log2(x)), while floats always use 32 bits (or 64 for double precision).

The input numbers (floating point) are encoded in a "fixed point" representation,
first computing some deltas (as they are all assumed to come from nearer spatial
coordinates). Then a fixed exponent is shared across all deltas, and each of them
is represented as a mantissa. See delta_int_encode implementation for details.

This idea is extensible also for coordinates by simply encoding X's and Y'x separately;
for this the wrapper functions delta_int_encode_points and delta_int_decode_points
are implemented.

Note that protobuf messages are not part of these functions, but it is trivial
to serialize encoded values in any message with 3 fields of type
(float, repeated sint32, sint32).

NOTE: This module is also implemented in nodejs codebase: arrayUtils.js
"""


def delta_int_encode(values, max_bits=15, compute_stats=False):
    """
    Encode a list of floating point values in DeltaInt form (see this file's header).

    The return value is a 4-element tuple:
    - A float `anchor`
    - A list of integer `deltas` (as a difference from `anchor`), scaled up or down by a given
      power of 2: 2^p
    - An exponent `p` which determines how to scale `deltas` back.
      This exponent is set to be 0. The return value 0 is reserved to represent an empty
      input array (since protobuf cannot represent None or null values)
    - An object with some stats if compute_stats is True, or None otherwise.
      This object includes for now values avg_abs_error and avg_rel_error with the (averages)
      absolute and relative errors that would result from decoding the received values.
      More keys could be added to this object in the future.

    Argument max_bits controls how many significant bits are kept to represent each
    output integer (the deltas). The higher maxBits value, the more precision is obtained
    when decoding values. But also, the deltas will be larger numbers, meaning their
    protobuf encoding will occupy more bits (roughly: maxBits).

    :param values is a list or sequence of floating point numbers
    :param max_bits controls the magnitude of returned deltas (more bits: more precision
    in the decoded results; but also more space occupied in protobuf serialization)
    """
    # When the input is an empty array, skip all the math below
    if len(values) == 0:
        return (0, [], 0)
    # The anchor is chosen to generate small distances to all other values
    anchor = sum(values) / len(values)  # empty list case is handled above
    # Estimate the maximum magnitude to represent, as the difference between
    # the minimum and maximum deltas.
    magnitude = abs(max(values, key=lambda x: abs(x - anchor)) - anchor)
    # Find the exponent that brings values closest to 2^max_bits
    power = 1 << max_bits
    if magnitude <= 0:
        scale = 0
        exponent = 0
    elif power > magnitude:
        # Max value is less than 2^max_bits, so we will scale up numbers before casting to int
        exponent = 1
        while power > magnitude * (1 << exponent):
            exponent += 1
        scale = 1 << exponent
    else:
        # Max value is greater than 2^max_bits, so we will scale down numbers before casting to int
        exponent = -1
        while power < magnitude / (1 << -exponent):
            exponent -= 1
        scale = 1 / (1 << -exponent)
    # Finally scale up (or down) all deltas with multiplying or dividing by 2^abs(exponent)
    int_deltas = [int(round((x - anchor) * scale)) for x in values]
    if not compute_stats:
        return (anchor, int_deltas, exponent, None)
    else:
        # Compute and return some statistics in the last tuple element.
        # These calculations should not be enabled by default.
        # For any calculation below, always prefer NOT allocating memory (use iterators)
        avg_abs_error = 0
        avg_rel_error = 0
        for v, d in zip(values, int_deltas):
            # delta / scale + anchor is the decoding function (see delta_int_decode)
            decoded = d / scale + anchor
            avg_abs_error += abs(decoded - v)
            avg_rel_error += abs((decoded - v) / v)
        avg_abs_error /= len(values)
        avg_rel_error /= len(values)
        stats = {
            "avg_abs_error": avg_abs_error,
            "avg_rel_error": avg_rel_error,
        }
        return (anchor, int_deltas, exponent, stats)


def delta_int_decode(anchor, deltas, exponent, options=None):
    """
    Decodes a DeltaInt encoding (see above) given as a tuple back into an float array.

    Note that the three arguments are exactly the elements returned by delta_int_encode;
    there is normally no need to manipulate them explicitly.

    :param anchor A floating point number (normally the mean of the input numbers)
    :param deltas A list of integers being the differences between each value and the anchor;
    representing fixed-point numbers
    :param exponent An value to scale all deltas by 2^exponent (fixed point representation)
    """
    # Determine scaling used and re-build deltas with scaling and adding back
    # the anchor value to become points.
    # This up- or down-scales values depending if exponent is greater or less than 0
    scale = 1 << exponent if exponent >= 0 else 1 / (1 << -exponent)
    values = [d / scale + anchor for d in deltas]
    return values


def delta_int_encode_points(points, max_bits=10, compute_stats=False):
    """
    Encodes a list of points { x, y } using DeltaInt
    and returns the 2 encodings as a pair (each element is a DeltaInt tuple, see above).

    :param points is a list or sequence of points (x, y)
    :param max_bits controls the magnitude of returned deltas (more bits: more precision
    in the decoded results; but also more space occupied in protobuf serialization)
    """
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    return (
        delta_int_encode(xs, max_bits, compute_stats),
        delta_int_encode(ys, max_bits, compute_stats),
    )


def delta_int_decode_points(encoded_xs, encoded_ys):
    """
    Decodes two DeltaInt encodings (see above) of the same length into a list of pairs (x, y).

    :param encoded_xs An encoding of all X coordinates
    :param encoded_ys An encoding of all Y coordinates
    """
    xs = delta_int_decode(*encoded_xs)
    ys = delta_int_decode(*encoded_ys)
    return list(zip(xs, ys))

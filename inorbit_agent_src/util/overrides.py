# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
"""
Convenience decorator to validate proper method inherittance
Taken from https://stackoverflow.com/a/8313042
"""


def overrides(interface_class):
    def overrider(method):
        assert method.__name__ in dir(interface_class)
        return method

    return overrider

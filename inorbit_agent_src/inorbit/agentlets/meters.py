# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
import time


class Meters:

    """
    A Meters object saves an snapshot of some counters at a given timestamp,
    and can perform basic arithmetic on them -- e.g. calculating a delta
    over two different Meters snapshots. Moreover, Meters are created with a
    definition (a list of meter names) and consistency is ensured across
    accesses: a valid meter value can be absent, but trying to set or get
    a meter name not in the Meters definition raises an error. Better than
    just using a raw dictionary!

    TODO(herchu) At some point this class will be able to "serialize" into
    an mqtt packet.

    Members:

    _names: dictionary whose keys are meter names.
    TODO(herchu) expand these 'schema' definition with datatypes,
    e.g. { "count": int, "weight": float } (values are unused)

    _values: meter values. Keys are also in _names. Some may be missing

    _ts: timestamp (seconds since epoch)

    _duration: Only for "delta" Meters; the difference in time between the
    two Meters compared

    """

    def __init__(self, ts=None, template=None, names=None):
        """
        Constructs a Meters object.
        It receives a timestamp (optional) and one of two optional parameters:
        - a meters object, used as template (only for meters names)
        - a list or a dictionary of meter names

        The Meters object starts with no values (not the same as being zero
        valued).
        """

        self._ts = ts if (ts is not None) else time.time()
        self._values = {}
        if template is not None:
            self._names = template._names
        elif names is not None:
            if isinstance(names, list):
                self._names = dict.fromkeys(names)
            else:
                self._names = names  # A dictionary
        else:
            raise Exception("Incomplete meters definition; lacking meter names")

    def set(self, name, value):
        """
        Sets the value of a meter.
        """

        if name not in self._names:
            raise Exception("set: Invalid meter name")
        self._values[name] = value

    def get(self, name, default=0):
        """
        Gets the value of a meter.
        """

        if name in self._values:
            return self._values[name]
        elif name not in self._names:
            raise Exception("get: Invalid meter name")
        else:
            return default

    def ts(self):
        """
        Gets the timestamp (float value, python 'time' module).
        """

        return self._ts

    def ts_as_ms(self):
        """
        Gets the timestamp (int value, milliseconds).
        """

        return int(self.ts() * 1000)

    def duration(self):
        """
        Returns the duration (in seconds, float value), only if this is a delta
        object - otherwise it returns None.
        """

        return self._duration

    def delta(self, other):
        """
        Constructs a new Meters object as the delta from 'other' to 'this' object.
        """

        m = Meters(self._ts, template=self)
        for k in self._names:
            m.set(k, self._values[k] - other.get(k, 0))
        m._duration = self._ts - other._ts
        return m

    def is_empty(self):
        """
        Tells if this Meters object holds no values.
        """

        return len(self._values) == 0

    def __str__(self):
        """
        String representation - for debugging.
        """

        return (
            "{@"
            + str(self._ts)
            + ": "
            + "; ".join(
                [
                    k + "=" + (str(self._values[k]) if (k in self._values) else "?")
                    for k in self._names
                ]
            )
            + "}"
        )

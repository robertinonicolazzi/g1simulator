# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
# Utility classes for working with threads
import threading


class Interval(threading.Thread):
    def __init__(self, callback, seconds, *args, **kwargs):
        """
        New thread that runs a callback every given seconds.
        """

        self._callback = callback
        self.__stop = threading.Event()
        self._seconds = seconds
        super(Interval, self).__init__(target=self._run, args=args, kwargs=kwargs)

    def _run(self, *args, **kwargs):
        """
        Overwrites threading.Thread()._run method with an infinite loop
        of the callback function spaced by the amount of seconds passed to the
        constructor.
        """

        # Run the callback one time immediately
        self._callback(*args, **kwargs)
        # and every self._seconds after that
        while not self.__stop.wait(self._seconds):
            self._callback(*args, **kwargs)

    def start(self):
        """
        Overwrites threading.Thread().start method. The only difference is that
        this method also returns the instance.
        """

        super(Interval, self).start()
        return self

    def stop(self):
        """
        Stops the thread by setting the stop event and breaking the while loop
        in _run method.
        """

        self.__stop.set()

    def set_interval_seconds(self, interval_seconds):
        """
        Changes the interval at which this Interval thread ticks. It takes effect after the next
        callback call; it does not cancel the current wait()
        """
        self._seconds = interval_seconds

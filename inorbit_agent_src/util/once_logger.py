# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Once logger class. Used when tracking exceptions that might occur at a
# high rate, logging them only once. Keeps track of the number of occurrences
# of each exception and allows three log levels: warn, error, exception.


class OnceLogger:
    def __init__(self, log_handle):
        self._log = log_handle
        # Dictionary of number of occurrences of each exception, indexed by
        # event id.
        self._reported_events = {}

    def warn(self, key, msg):
        """
        Logs a warn message if it's a new event, and increments occurrence
        number if it's a known issue.

        TODO (FlorGrosso): rename this method to 'warning' to be consistent
        with inorbit's logger naming convention.
        """

        if self._should_report_event(key):
            self._log.warning(msg + " [reported once]")
            self._reported_events[key] = 1
        else:
            self._reported_events[key] += 1

    def error(self, key, msg):
        """
        Logs an error message if it's a new event, and increments occurrence
        number if it's a known issue.
        """

        if self._should_report_event(key):
            self._log.error(msg + " [reported once]")
            self._reported_events[key] = 1
        else:
            self._reported_events[key] += 1

    def exception(self, key, msg):
        """
        Logs an exception message if it's a new event, and increments occurrence
        number if it's a known issue.
        """

        if self._should_report_event(key):
            self._log.exception(msg + " [reported once]")
            self._reported_events[key] = 1
        else:
            self._reported_events[key] += 1

    def reset_one(self, key):
        """
        Resets event tracking for a particular event.
        """

        if self._reported_events.get(key):
            self._reported_events[key] = 0

    def reset_set(self, keys):
        """
        Resets event tracking for a set of events.
        """

        map(self.reset_one, keys)

    def reset_all(self):
        """
        Resets event tracking for the whole module.
        """

        self._reported_events = {}

    def _should_report_event(self, key):
        """
        Checks if an event should be reported, based on its number of occurrence.
        """

        return self._reported_events.get(key, 0) == 0

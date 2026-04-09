# Copyright (c) 2022, InOrbit, Inc.
# All rights reserved.
# Utility class superclass to track network usage (bytes transmitted and received)


class NetworkingMixin:
    def __init__(self):
        self._tx_bytes = 0
        self._rx_bytes = 0

    def get_tx_bytes(self):
        """
        Returns total number of bytes transmitted since the object was created
        """

        return self._tx_bytes

    def get_rx_bytes(self):
        """
        Returns total number of bytes received since the object was created
        """

        return self._rx_bytes

    def add_tx_bytes(self, bytes):
        self._tx_bytes += bytes

    def add_rx_bytes(self, bytes):
        self._rx_bytes += bytes

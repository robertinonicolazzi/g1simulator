# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agentlet that provides system information (CPU load, network stats, HDD usage)
import os
import subprocess
import threading
import time

import psutil
from util.overrides import overrides

from .agentlet import Agentlet
from .inorbit_pb2 import DiskUsageMessage
from .inorbit_pb2 import NetworkStatsMessage
from .inorbit_pb2 import SystemStatsMessage
from .meters import Meters


# Internal keys for the Meters object (any string works)
TOTAL_TX = "t_tx"
TOTAL_RX = "t_rx"
INORBIT_TX = "o_tx"
INORBIT_RX = "o_rx"
MQTT_TX = "m_tx"
MQTT_RX = "m_rx"
SLEEP_INTERVAL_S = 10

# If hdd source is undefined, compute usage from all the aggregated storage in
# the robot
HDD_SOURCE_DEFAULT = "/"


class SystemAgentlet(Agentlet):
    def __init__(self, uplink, diagnostics):
        super(SystemAgentlet, self).__init__(uplink)
        self._loop_thread = None
        self._run = False
        self._inorbit_path = os.getenv(
            "INORBIT_HOME", os.path.join(os.path.expanduser("~"), ".inorbit")
        )
        self._stats = None
        self._diagnostics = diagnostics

        self._detailed_net_stats = {}

        # Primary disk to report usage of
        self._primary_disk_partition = HDD_SOURCE_DEFAULT
        # Optional disk sources to report. This is a dictionary of partition
        # paths indexed by volume id.
        self._states["optional_disk_sources"] = {}
        # Optional network interfaces to monitor. This is a dictionary of
        # mount points indexed by interfaces ids.
        self._states["optional_network_interfaces"] = {}

        # List of disk partitions names
        self._states["available_hdd_partitions"] = self._list_disk_partitions()

        # List of network interfaces
        self._states["available_network_interfaces"] = self._list_network_interfaces()

        # Initialize the list of "networking" modules (those who transmit/receive data) with
        # the MQTT uplink, which is always metered
        self._networking_modules = [uplink]

    @overrides(Agentlet)
    def load(self, runlevel):
        self._run = True
        self._loop_thread = threading.Thread(target=self._loop_main)
        self._loop_thread.start()
        self._states["loaded"] = True
        self._load_states()

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return self._states["loaded"]

    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """

        if "optional_disk_sources" in state.keys():
            # Sanity checks. If the state received is not a dictionary,
            # skip it.
            if isinstance(state["optional_disk_sources"], dict):
                self._states["optional_disk_sources"] = state["optional_disk_sources"]
                # Restart exception reporting
                self.once_logger.reset_all()

        if "optional_network_interfaces" in state.keys():
            if isinstance(state["optional_network_interfaces"], dict):
                self._states["optional_network_interfaces"] = state["optional_network_interfaces"]
                self.once_logger.reset_all()

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)

    def get_stats(self):
        return self._stats

    def _loop_main(self):
        """
        Main agentlet thread.
        Periodically reads and publishes system information.
        """

        self.logger.info("Starting system agentlet main loop.")

        # Initial (empty) Meters object to hold the metrics names
        # No timestamp, this object is not sent
        meters = Meters(names=self._get_meters_names())

        while self._run:
            metersNow = Meters(template=meters)
            # Read total CPU load average within a small interval
            # No interval=x in this call; CPU load since last call
            load_average = psutil.cpu_percent()

            # Get HDD usage (from all the aggregated storage in the robot)
            total_disk_usage = self._compute_disk_usage_for(HDD_SOURCE_DEFAULT)

            # Get the storage used by InOrbit
            inorbit_hdd_usage_mb, inorbit_hdd_usage_percentage = self._compute_inorbit_disk_usage(
                total_disk_usage
            )

            # Get network I/O statistics per interface
            net = self._get_detailed_net_stats()

            # Total network metrics
            total_net = self._get_total_net_stats_from(net)
            metersNow.set(TOTAL_TX, total_net[0])
            metersNow.set(TOTAL_RX, total_net[1])
            # InOrbit network metrics
            metersNow.set(INORBIT_TX, self.get_inorbit_tx_bytes())
            metersNow.set(INORBIT_RX, self.get_inorbit_rx_bytes())
            metersNow.set(MQTT_TX, self.uplink.get_tx_bytes())
            metersNow.set(MQTT_RX, self.uplink.get_rx_bytes())
            # Optional network interfaces metrics
            self._set_optional_net_meters_from(net, metersNow)

            # Get RAM usage - percentage in [0...1] range
            memory_usage_percent = self._get_ram_usage_percent()

            if not meters.is_empty():  # Report the delta - only done
                # ... after a second pass on this loop
                delta = metersNow.delta(meters)
                # Publish all stats (in a single message)\
                self._stats = SystemStatsMessage()
                self._stats.timestamp = delta.ts_as_ms()
                self._stats.elapsed_seconds = delta.duration()
                self._stats.cpu_load_percentage = load_average / 100
                if total_disk_usage:
                    self._stats.hdd_usage_percentage = total_disk_usage.percent / 100
                self._stats.inorbit_hdd_usage_mb = inorbit_hdd_usage_mb
                self._stats.inorbit_hdd_usage_percentage = inorbit_hdd_usage_percentage
                self._stats.optional_disks_data.extend(self._create_optional_disks_msg_array())
                self._stats.optional_network_interfaces_data.extend(
                    self._optional_net_interfaces_msg_from_delta(delta)
                )
                self._stats.total_tx = delta.get(TOTAL_TX)
                self._stats.total_rx = delta.get(TOTAL_RX)
                self._stats.inorbit_tx = delta.get(INORBIT_TX)
                self._stats.inorbit_rx = delta.get(INORBIT_RX)
                self._stats.mqtt_tx = delta.get(MQTT_TX)
                self._stats.mqtt_rx = delta.get(MQTT_RX)
                self._stats.ram_usage_percentage = memory_usage_percent
                self.uplink.publish_protobuf("system/stats", self._stats)

            # Save last timestamp and sleep for a while
            meters = metersNow
            time.sleep(SLEEP_INTERVAL_S)

        self.logger.info("Ending system agentlet main loop.")

    def _get_meters_names(self):
        """
        Returns the list of names used to build a meters template.
        """

        names = [TOTAL_TX, TOTAL_RX, INORBIT_TX, INORBIT_RX, MQTT_TX, MQTT_RX]

        for net_interface in self._states["available_network_interfaces"]:
            names.extend([f"{net_interface}_TX", f"{net_interface}_RX"])

        return names

    def _load_states(self):
        """
        Sets initial state values when loading the agentlet.
        """

        # No state loaded (battery handling removed)
        pass

    def _list_disk_partitions(self):
        """
        Returns a list with the mountpoints for disk partitions available.
        """

        return [partition.mountpoint for partition in psutil.disk_partitions()]

    def _compute_disk_usage_for(self, partition):
        """
        Returns disk usage statistics about the partition which contains the given
        path as a named tuple including total, used and free space expressed in
        bytes, plus the percentage usage.
        """

        # Get HDD usage from the source specified under states
        try:
            return psutil.disk_usage(partition)
        except Exception as e:
            self.once_logger.warn(
                f"disk_usage for {partition}",
                f"Failed to compute disk usage for partition: '{partition}'.",
            )

            return None

    def _compute_inorbit_disk_usage(self, primary_disk_data):
        """
        Returns inorbit's hdd usage in mb and percentage. This last value is
        calculated based on the total space of the primary disk.
        """

        # TODO (Flor_Grosso): Implement a proper locking mechanism to
        # avoid race conditions while computing hdd usage and deleting
        # data files under the .inorbit directory.
        inorbit_hdd_usage_mb = 0
        inorbit_hdd_usage_percentage = 0

        try:
            with open(os.devnull, "w") as devnull:
                inorbit_hdd_usage = float(
                    subprocess.check_output(
                        ["du", "-sb", self._inorbit_path], stderr=devnull
                    ).split()[0]
                )
            # Convert bytes to MB
            inorbit_hdd_usage_mb = inorbit_hdd_usage / (1024 * 1024)
            # Get inorbit disk usage as a percentage of disk's total
            if primary_disk_data and primary_disk_data.total > 0:
                inorbit_hdd_usage_percentage = inorbit_hdd_usage / primary_disk_data.total
        except Exception as e:
            self.once_logger.warn("inorbit_disk_usage", "Could not compute inorbit disk usage.")

        return inorbit_hdd_usage_mb, inorbit_hdd_usage_percentage

    def _create_optional_disks_msg_array(self):
        """
        Returns a list of DiskUsageMessage() elements, which contains information
        about optional disk partitions (id and usage percents).
        """

        optional_disk_data = []

        for volume_id in self._states["optional_disk_sources"]:
            volume_path = self._states["optional_disk_sources"][volume_id]
            disk_usage = self._compute_disk_usage_for(volume_path)

            if disk_usage:
                data = DiskUsageMessage()
                data.volume_id = volume_id
                # Don't send a message with empty data
                if not disk_usage.total:
                    continue
                data.usage_percentage = disk_usage.percent / 100
                optional_disk_data.append(data)

        return optional_disk_data

    def _list_network_interfaces(self):
        """
        Returns a list with the names of the network interfaces available.
        """

        return list(psutil.net_if_addrs().keys())

    def _get_detailed_net_stats(self):
        """
        Returns dictionary of complete network stats per available interface.
        """

        return psutil.net_io_counters(pernic=True)

    def _get_total_net_stats_from(self, net_stats):
        """
        Returns the total of bytes sent and received by all interfaces.
        """

        bytes_sent = 0
        bytes_recv = 0

        for interface in net_stats:
            # Get data traffic for all interfaces but localhost
            if interface != "lo":
                bytes_sent += net_stats[interface].bytes_sent
                bytes_recv += net_stats[interface].bytes_recv

        return (bytes_sent, bytes_recv)

    def _get_net_stats_for(self, detailed_stats, interface):
        """
        Returns the number of bytes sent and received by a specific interface
        """

        bytes_sent = 0
        bytes_recv = 0

        interface_stats = detailed_stats.get(interface)

        if interface_stats:
            bytes_sent = detailed_stats[interface].bytes_sent
            bytes_recv = detailed_stats[interface].bytes_recv

        return (bytes_sent, bytes_recv)

    def _set_optional_net_meters_from(self, detailed_net_stats, meters):
        """
        Sets meters values for optional network interfaces
        (bytes sent and bytes received per id) from the current net stats.
        """

        for net_interface in self._states["available_network_interfaces"]:
            (bytes_sent, bytes_recv) = self._get_net_stats_for(detailed_net_stats, net_interface)
            meters.set(f"{net_interface}_TX", bytes_sent)
            meters.set(f"{net_interface}_RX", bytes_recv)

    def _optional_net_interfaces_msg_from_delta(self, delta):
        """
        Returns a list of NetworkStatsMessage() elements, which contain information
        about optional network interfaces (id, bytes sent and bytes received).
        Numeric values are obtained from the deltas of the previous pass.
        """

        optional_network_interfaces_data = []

        for net_interface in self._states["optional_network_interfaces"]:
            mountpoint = self._states["optional_network_interfaces"][net_interface]

            try:
                bytes_sent = delta.get(f"{mountpoint}_TX")
                bytes_recv = delta.get(f"{mountpoint}_RX")

                if bytes_sent or bytes_recv:
                    data = NetworkStatsMessage()
                    data.interface_id = net_interface
                    data.tx = bytes_sent
                    data.rx = bytes_recv
                    optional_network_interfaces_data.append(data)
            except Exception as e:
                self.once_logger.warn(
                    f"net_stats for {mountpoint}",
                    "Attempt to get network stats for a non-available "
                    f"interface: '{mountpoint}'.",
                )

        return optional_network_interfaces_data

    def _get_ram_usage_percent(self):
        """
        Computes and returns RAM usage percent in a range from 0 to 1, based on
        total and available memory stats.
        """

        try:
            memory_stats = psutil.virtual_memory()
            used_memory = memory_stats.total - memory_stats.available
            return float(used_memory) / memory_stats.total
        except Exception as e:
            return 0

    def register_networking_module(self, module):
        """
        Registers a new module (normally, an Agentlet) as a module that uses and tracks use of
        networking: an util.NetworkingMixin object. It will be queried to report the total
        number of bytes transmitted and received by InOrbit agent.
        """

        self._networking_modules.append(module)
        # List the tracked modules in a module state, mostly for server-side debugging to know
        # which modules are already enabled (and tracked) by each agent.
        self._states["networking_modules"] = [
            m.__class__.__name__ for m in self._networking_modules
        ]

    def get_inorbit_tx_bytes(self):
        """
        Returns total number of bytes transmitted by InOrbit agent. This includes MQTT traffic
        and also traffic from other modules registered in register_networking_module()
        """

        return sum([m.get_tx_bytes() for m in self._networking_modules])

    def get_inorbit_rx_bytes(self):
        """
        Returns total number of bytes received by InOrbit agent. This includes MQTT traffic
        and also traffic from other modules registered in register_networking_module()
        """

        return sum([m.get_rx_bytes() for m in self._networking_modules])

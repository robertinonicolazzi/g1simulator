# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Alert Manager agentlet.
#
import threading
import time
from builtins import int

import agentlet
from inorbit_pb2 import AlertMessage
from util.overrides import overrides

ALERT_PUBLISH_RATE_HZ = 1
DIAGNOSTIC_LEVEL_OK = 0
DIAGNOSTIC_LEVEL_WARN = 1
DIAGNOSTIC_LEVEL_ERROR = 2
DIAGNOSTIC_LEVEL_STALE = 3

MQTT_ALERT_TOPIC = "alerts/out"


class AlertManagerAgentlet(agentlet.Agentlet):
    def __init__(self, uplink, diagnostics):
        super(AlertManagerAgentlet, self).__init__(uplink)
        self._diagnostics = diagnostics
        # Dictionary of diagnostics component ids ("RosDiag:" + alert.name)
        # to level
        self._alerts_history = {}
        self._diagnostics_msgs = []
        self._diagnostics_ts = int(0)

        # Mutex used to access _alerts_history.
        self._mutex = threading.Lock()

        # Publisher thread running state
        self._alert_publisher_running = False

    @overrides(agentlet.Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel

        # Start publishing thread
        threading.Thread(target=self._publish_loop).start()

        self._states["loaded"] = True
        return True

    @overrides(agentlet.Agentlet)
    def unload(self):
        self._states["loaded"] = False
        self._alert_publisher_running = False
        self.once_logger.reset_all()
        return True

    @overrides(agentlet.Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel

    def _build_alert(self, diagnostic_name, ts):
        alert = AlertMessage()
        alert.ts = ts
        alert.name = diagnostic_name
        alert.level = str(self._diagnostics_msgs[diagnostic_name].get("level"))
        alert.message = self._diagnostics_msgs[diagnostic_name].get("msg")
        alert.component_id = "RosDiag:" + alert.name.strip()
        return alert

    def _is_not_alert(self, level, component_id):
        """
        Returns True if the alert is either a known issue (present in alerts
        history, with the same level) or if it's stale.
        """

        message_is_known = (
            component_id in self._alerts_history and self._alerts_history[component_id] == level
        )
        message_is_stale = int(level) == DIAGNOSTIC_LEVEL_STALE
        return message_is_known or message_is_stale

    def _is_new_alert(self, level, component_id):
        """
        Returns True if the level is other than OK and the component id
        wasn't reported/ was OK before.
        """

        return int(level) > DIAGNOSTIC_LEVEL_OK and self._alerts_history.get(component_id) != level

    def _is_alert_cleared(self, level, component_id):
        """
        Returns True if a component id previously reported as error, has
        now changed to level OK.
        """

        return int(level) == DIAGNOSTIC_LEVEL_OK and self._alerts_history.get(component_id) != level

    def _parse_diagnostic_msg(self):
        diagnostics_data = self._diagnostics.get_diagnostics()

        if diagnostics_data:
            self._diagnostics_msgs = diagnostics_data.get("data", {})
            self._diagnostics_ts = diagnostics_data.get("ts", int(0))

    def _publish_loop(self):
        self._alert_publisher_running = True
        while self._alert_publisher_running:
            try:
                self._publish_diagnostics_if_available()
            except Exception as e:
                self.once_logger.exception("alerts_publish", "Exception publishing data.")
            time.sleep(1.0 / ALERT_PUBLISH_RATE_HZ)

        self.logger.info("Publisher thread shutting down.")

    def _publish_diagnostics_if_available(self):
        self._parse_diagnostic_msg()

        if len(self._diagnostics_msgs) > 0:
            with self._mutex:
                for component in self._diagnostics_msgs:
                    # Ignore empty diagnostics data
                    if not self._diagnostics_msgs.get(component):
                        continue

                    alert_data = self._build_alert(component, self._diagnostics_ts)
                    level = alert_data.level
                    component_id = alert_data.component_id

                    if self._is_not_alert(level, component_id):
                        continue

                    if self._is_new_alert(level, component_id):
                        alert_data.status = "new"

                    elif self._is_alert_cleared(level, component_id):
                        alert_data.status = "cleared"

                    # Update alerts history
                    self._alerts_history[component_id] = level

                    self.uplink.publish_protobuf(MQTT_ALERT_TOPIC, alert_data, qos=1)

    def _trigger_alerts_resend(self):
        """
        Forces the module to resend alerts by cleaning up alerts history.
        """

        with self._mutex:
            self._alerts_history = {}
        self.logger.info("Triggering a diagnostics alerts resend")

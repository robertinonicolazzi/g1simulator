# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Log Manager class.
import glob
import os
import threading
import time
from queue import Queue

from inorbit.agentlets.inorbit_pb2 import RobotFileData
from inorbit.agentlets.inorbit_pb2 import RobotFilesUpdateMessage
from inorbit.logger import getLog
from inorbit.logger import INORBIT_LOG_PATH
from util.artifacts import Artifact
from util.concurrency import Interval


MQTT_UPDATES_TOPIC = "logfiles_update"
LOG_FILE_TYPE = "log"
UPLOAD_FILE_RATE_HZ = 1.0


class LogManager(object):
    def __init__(self, uplink):
        self._link = uplink
        self.logger = getLog(__name__)
        self._logs = {}
        self._logs_mutex = threading.Lock()
        self._upload_requests_queue = Queue()
        self._uploader_thread = None

    def upload_log(self, file_name):
        """
        Adds an upload request to the queue, when an upload command is received.
        """

        if not self._uploader_thread:
            self._uploader_thread = Interval(self._upload_logs, 1.0 / UPLOAD_FILE_RATE_HZ).start()
        self._upload_requests_queue.put(file_name)

    def _create_update_msg(self):
        """
        Returns a RobotFilesUpdateMessage, made from the current logs.
        """

        files_update = RobotFilesUpdateMessage()
        files_messages = []
        with self._logs_mutex:
            for log in self._logs.values():
                file_msg = RobotFileData()
                file_msg.type = LOG_FILE_TYPE
                file_msg.name = log.name
                file_msg.stored_in_robot = log.in_robot
                file_msg.uploading_to_cloud = log.uploading
                file_msg.url = log.url
                file_msg.ts = log.ts
                file_msg.size = log.size_kb
                files_messages.append(file_msg)

        files_update.artifacts.extend(files_messages)
        files_update.ts = int(time.time() * 1000)

        return files_update

    def _publish_logs_update(self):
        """
        Publishes an update message using protobuf. The whole log list is sent
        back again.
        """

        self._link.publish_protobuf(MQTT_UPDATES_TOPIC, self._create_update_msg(), qos=1)

    def send_list_update(self):
        """
        Cleans up current file handlers and refills it with current data to publish
        it through protobuf.
        NOTE: this is specific for log files, due to the way we are handling UI
        updates.
        """

        self._sync_logs()
        self._publish_logs_update()

    def _sync_logs(self):
        """
        Returns a dictionary of robot log files, indexed by file name.
        """

        current_logs = glob.glob(os.path.join(INORBIT_LOG_PATH, "*.log*"))
        with self._logs_mutex:
            self._logs = {}
            for filepath in current_logs:
                filename = os.path.basename(filepath)
                self._logs[filename] = Artifact(filepath, compress=True)

    def _upload_logs(self):
        """
        Runs on a separate thread. Checks uploads requests buffer and uploads the
        logs if possible.
        """

        while self._upload_requests_queue.qsize() != 0:
            filename = self._upload_requests_queue.get()
            self.logger.info("Uploading %s to cloud.", filename)
            try:
                self._logs[filename].upload()
            except Exception as e:
                self.logger.exception("Could not upload %s to cloud.", filename)
                # Check that the file exists before updating its uploading status
                if filename in self._logs:
                    self._logs[filename].set_uploading(False)
            # Send update to cloud
            self._publish_logs_update()

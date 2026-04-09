# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# TopicMonitor class. Keeps a list of rostopics to monitor, subscribes to them
# and performs operations to get requested parameters (rate).
import threading

import inorbit.logger
from inorbit.agentlets.inorbit_pb2 import TopicMonitorMessage
from util.once_logger import OnceLogger


class TopicMonitor:
    def __init__(self, ros, topics_to_monitor):
        self._ros = ros
        self.logger = inorbit.logger.getLog(self.__class__.__name__)
        # Logger for high rate exceptions
        self.once_logger = OnceLogger(self.logger)

        # Dictionary of topic info objects types indexed by topic names.
        self._topic_info = {}

        # Monitoring state
        self._monitoring = False

        # List of available topic names.
        self._available_ros_topics = []

        # Dictionary of actions to perform indexed by topic name
        self._topics_to_monitor = topics_to_monitor

        # Mutex used to access _topics_to_monitor
        self._mutex = threading.Lock()

        # Flag that indicates whether a dependencies load request has been
        # already issued.
        self._has_tried_to_load_dependencies = False

    def load_dependencies(self):
        """
        Tries to load topic info module and returns False if it fails.
        """

        # Avoid loading dependencies more than once
        if self._has_tried_to_load_dependencies:
            return "TopicInfo" in globals()

        # Set flag to true after first run
        self._has_tried_to_load_dependencies = True

        try:
            from util.topic_info import TopicInfo

            global TopicInfo
        except Exception as e:
            self.logger.warning(f"Unable to monitor topics: {e}")
            return False
        return True

    def start(self):
        """
        Starts monitoring processes over the list of topics to monitor.
        """

        for topic in self._topics_to_monitor:
            # Get the topic type and setup the TopicInfo.
            topic_type = self._ros.get_msg_type_for(topic)

            self._topic_info[topic] = TopicInfo(topic, topic_type=topic_type)
            self._topic_info[topic].start_monitoring()
        self._monitoring = True

    def stop(self):
        """
        Stops monitoring topics.
        """

        if self._monitoring:
            for topic in self._topics_to_monitor:
                self._topic_info[topic].stop_monitoring()
            self._monitoring = False

    def update_topics(self, new_topics):
        """
        Updates topics to monitor with the of new topics received. This is a
        dictionary which of actions to perform indexed by topic name.

        NOTE (Flor_Grosso): actions to perform are not configurable for now and the
        same one (default: rate) is used for all topics.

        NOTE 2 (Flor_Grosso): to accomplish this, monitoring processes are stopped
        before and then resumed later with the new list.

        TODO (Flor_Grosso): implement the logic to update topics, leaving those
        that will continue to be monitored unchanged, removing the deleted ones
        and adding the appended ones.
        """

        should_update = self._monitoring
        if should_update:
            self.stop()

        with self._mutex:
            self._topics_to_monitor = new_topics

        if should_update:
            self.start()

        # Reset exceptions tracking
        self.once_logger.reset_all()

    def make_update_msg(self):
        """
        Returns a list of TopicMonitorMessage, for the current topics being
        monitored.
        """

        topic_update_array = []

        # TODO (Flor_Grosso): Use only keys for a start. Consider checking
        # values and iterating among the actions to perform on each item.
        with self._mutex:
            topics_to_monitor = self._topics_to_monitor.keys()

        for topic in topics_to_monitor:
            item = TopicMonitorMessage()
            item.name = topic
            try:
                rate = self._get_topic_rate(topic)
                item.has_rate = True
                item.rate = int(round(rate * 1000))
                topic_update_array.append(item)
            except Exception as e:
                self.once_logger.warn(topic, f"Exception monitoring topic '{topic}'.")
                continue
        return topic_update_array

    def _get_topic_rate(self, topic):
        """
        Returns topic rate in Hz.
        """

        rate = self._topic_info.get(topic).get_hz()[0]
        return rate if rate is not None else 0

    def list_available_topics(self):
        """
        Returns a list of all the available topic names.
        """

        topics = self._ros.get_published_topics()
        if topics:
            return [item[0] for item in topics]

        return []

    def _create_topics_dict(self):
        """
        Returns a dictionary of topic types indexed by topic names.
        """

        available_ros_topics = self._ros.get_published_topics()
        topics_dict = {}

        if available_ros_topics:
            for topic in available_ros_topics:
                topics_dict[topic[0]] = topic[1]

        return topics_dict

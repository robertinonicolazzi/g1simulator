# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# The work in this file is based on topic_info:
# https://github.com/ros-visualization/rqt_topic/blob/master/src/rqt_topic/topic_info.py
# Parts of the code were taken and modified. The original license
# is reproduced below.
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from __future__ import division
from __future__ import with_statement

try:
    from cStringIO import StringIO as BufferType
except ImportError:
    from io import BytesIO as BufferType

try:
    import roslib
except Exception as e:
    raise Exception("Failed to load roslib")

try:
    import rospy
except Exception as e:
    raise Exception("Failed to load rospy")

try:
    import rostopic
except Exception as e:
    raise Exception("Failed to load rostopic")

import inorbit.logger
from once_logger import OnceLogger


class TopicInfo(rostopic.ROSTopicHz):
    def __init__(self, topic_name, topic_type=None):
        super(TopicInfo, self).__init__(100)
        self._topic_name = topic_name
        self._topic_type = topic_type

        # Last rate computed for the topic
        self._last_rate = 0

        self.logger = inorbit.logger.getLog(__name__)
        self.once_logger = OnceLogger(self.logger)
        self._subscriber = None
        self.monitoring = False
        self._reset_data()
        self.message_class = None

        # NOTE(Flor_Grosso): both rostopic.get_topic_type() and roslib.message.get_message_class()
        # break if the topic name doesn't start with "/", so we need to add it if the config
        # doesn't have it.
        # TODO(Flor_Grosso): the RosAgentlet does something similar in l.205. Consider moving this
        # functionality to a shared module.
        if self._topic_name is not None and self._topic_name[0] != "/":
            self._topic_name = "/" + self._topic_name

        try:
            self.message_class = roslib.message.get_message_class(self._topic_type)
        except Exception as e:
            self.message_class = None

    def _reset_data(self):
        self.last_message = None
        self.times = []
        self.timestamps = []
        self.sizes = []
        self._last_rate = 0

    def toggle_monitoring(self):
        if self.monitoring:
            self.stop_monitoring()
        else:
            self.start_monitoring()

    def start_monitoring(self):
        # If the message class is defined, then subscribe to the topic
        # providing the specific value.
        if self.message_class:
            self.monitoring = True
            self._subscriber = rospy.Subscriber(
                self._topic_name, self.message_class, self.message_callback
            )
        # If the message class is not known yet, use AnyMsg type and provide
        # a setup callback to find out this value.
        else:
            self.logger.info("Registering subscriber to: " + self._topic_name)
            self._subscriber = rospy.Subscriber(self._topic_name, rospy.AnyMsg, self.setup_callback)

    def stop_monitoring(self):
        self.monitoring = False
        self._reset_data()
        if self._subscriber is not None:
            self._subscriber.unregister()

    def setup_callback(self, message):
        """
        Callback for ros msgs of the type 'rospy.AnyMsg', used to query and set
        message class on the first msg received.
        """

        # Get topic class
        try:
            self._topic_type = rostopic.get_topic_type(self._topic_name)[0]
            self.message_class = roslib.message.get_message_class(self._topic_type)

            # Now reset monitoring to the corresponding message type
            self._subscriber.unregister()
            self.start_monitoring()
        except Exception as e:
            self.message_class = None
            self.once_logger.error(
                self._topic_name, f"Can't get msg class for '{self._topic_name}'."
            )

    def message_callback(self, message):
        """
        Callback for ros msgs with a known msg class.
        """

        rostopic.ROSTopicHz.callback_hz(self, message)
        with self.lock:
            self.timestamps.append(rospy.get_time())

            # FIXME: this only works for message of class AnyMsg
            # self.sizes.append(len(message._buff))
            # time consuming workaround...
            buff = BufferType()
            message.serialize(buff)
            self.sizes.append(len(buff.getvalue()))

            if len(self.timestamps) > self.window_size - 1:
                self.timestamps.pop(0)
                self.sizes.pop(0)
            assert len(self.timestamps) == len(self.sizes)

            self.last_message = message

    def get_bw(self):
        if len(self.timestamps) < 2:
            return None, None, None, None
        current_time = rospy.get_time()
        if current_time <= self.timestamps[0]:
            return None, None, None, None
        with self.lock:
            total = sum(self.sizes)
            bytes_per_s = total / (current_time - self.timestamps[0])
            mean_size = total / len(self.timestamps)
            max_size = max(self.sizes)
            min_size = min(self.sizes)
            return bytes_per_s, mean_size, min_size, max_size

    def get_hz(self):
        # If there's some rate data available, check if those values are valid
        # or have timed out, based on the last saved timestamp on ros callback.
        if self._last_rate > 0 and self.timestamps:
            # Timeout is set to 3 * topic period
            max_timeout = 3 / self._last_rate
            topic_has_timed_out = rospy.get_time() - self.timestamps[-1] > max_timeout

            if topic_has_timed_out:
                self._reset_data()

        if not self.times:
            return None, None, None, None
        with self.lock:
            n = len(self.times)
            mean = sum(self.times) / n
            rate = 1.0 / mean if mean > 0.0 else 0
            min_delta = min(self.times)
            max_delta = max(self.times)
            self._last_rate = rate
        return rate, mean, min_delta, max_delta

# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Spatial annotations agentlet.
# Depends on the ROS module.
import threading

import agentlet
import numpy as np
from ros import RosPublisher
from util.overrides import overrides

NO_GO_ZONE_COST = 100
PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 10
# Topic to publish ros data to, if none is provided on the config.
DEFAULT_OUTPUT_TOPIC = "inorbit/map"


class SpatialAnnotationsAgentlet(agentlet.Agentlet):
    def __init__(self, uplink, ros, localization):
        super(SpatialAnnotationsAgentlet, self).__init__(uplink)
        self._ros = ros
        self._localization = localization

        # ROS publisher for designed global maps/costmaps
        self._ros_map_publisher = RosPublisher()

        # Spatial annotations object. It contains publication params required
        # per publication mode and a description of the zones to trace (polygon
        # coordinates and cost to fill the areas with).
        #
        # Complete schema for this module state can be found under:
        # inorbit/web/lib/collections.js
        self._states["spatial_annotations"] = {}

        # Flag to indicate whether a map/costmap has already been published.
        self._costmap_published = False

        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()

    @overrides(agentlet.Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel

        try:
            from util.Image import Image

            global Image
        except Exception as e:
            self.once_logger.exception(
                "image_available", "Exception when loading Image package: " + str(e)
            )
            return False
        else:
            self.logger.info("Using %s as image processing package." % Image._PACKAGE)

        try:
            import nav_msgs
            import nav_msgs.msg

            global nav_msgs
        except Exception as e:
            self.once_logger.exception("nav_msgs_load", "Exception loading nav_msgs.")
            return False

        # Create the ROS publisher for costmap mode
        self._add_ros_submodule()

        threading.Thread(target=self._map_publisher).start()

        self._states["loaded"] = True

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)

        return True

    @overrides(agentlet.Agentlet)
    def unload(self):
        # Re-initialize exception reporting
        self.once_logger.reset_all()
        # Remove ROS subscribers
        self._ros.remove_submodule("spatial_annotations")
        self._costmap_published = False
        self._map_publisher_running = False
        self._states["loaded"] = False
        return True

    @overrides(agentlet.Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
        self.wake_up_publisher(self._condition)

    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """

        if "spatial_annotations" in state.keys():
            # First check if there's an output topic configured, or we should
            # use defaults.
            output_topic = (
                state["spatial_annotations"].get("publication_params", {}).get("output_topic")
            )
            if not output_topic:
                output_topic = DEFAULT_OUTPUT_TOPIC
                state["spatial_annotations"]["publication_params"]["output_topic"] = output_topic

            if self._states["loaded"]:
                # Update publisher with output topic
                self._update_ros_publisher(output_topic)
                self._costmap_published = False
                self.once_logger.reset_all()

            self._states["spatial_annotations"] = state["spatial_annotations"]

        # Send a state update
        self.publish_state(self.uplink, self._states)

    def _add_ros_submodule(self):
        """
        Adds the spatial_annotations submodule to ROS agentlet when loading. Note
        that the map publisher will be added if there's a valid state only.
        """

        output_topic = (
            self._states["spatial_annotations"]
            .get("publication_params", {})
            .get("output_topic", DEFAULT_OUTPUT_TOPIC)
        )

        if output_topic:
            self._ros.add_submodule(
                "spatial_annotations",
                pubs=((output_topic, nav_msgs.msg.OccupancyGrid, self._ros_map_publisher, True),),
            )
        else:
            self._ros.add_submodule("spatial_annotations")

    def _update_ros_publisher(self, new_topic):
        """
        Updates ros publisher when a new state is received.
        """

        # Do not update publisher if the new output topic is not valid or if it
        # didn't update.
        if (
            not new_topic
            or self._states["spatial_annotations"].get("publication_params", {}).get("output_topic")
            == new_topic
        ):
            return

        new_pub = (new_topic, nav_msgs.msg.OccupancyGrid, self._ros_map_publisher, True)
        self._ros.update_publisher_topic("spatial_annotations", new_topic, new_pub)

    def _map_publisher(self):
        """
        Map publisher loop. It runs on a separate thread.
        """

        self._map_publisher_running = True
        while self._map_publisher_running:
            try:
                self._maybe_publish()
            except Exception as e:
                self.once_logger.exception(
                    "spatial_annotations_publish", "Exception publishing data."
                )

            self._condition.acquire()
            self._condition.wait(PUBLISHER_PERIOD_DEFAULT_RUNLEVEL)
            self._condition.release()

        self.logger.info("Publisher thread shutting down.")

    def _maybe_publish(self):
        """
        Creates and publishes the map/costmap if possible and appropriate.
        """

        # If the costmap is already published, skip it
        if self._costmap_published:
            return

        # TODO (Flor_Grosso): Consider checking whether the source map data
        # received matches the localization config. Use
        # `_is_source_map_current()` for that.

        if self._ros_map_publisher.pub is None:
            self.once_logger.warn("ros_publisher_not_set", "ROS publisher not set. Aborting.")
            return

        publication_params = self._states["spatial_annotations"].get("publication_params", {})
        # Get current map to merge with the given polygons.
        source_topic = publication_params.get("source_topic")
        base_map = None

        try:
            base_map = self._ros.wait_for_message(source_topic, nav_msgs.msg.OccupancyGrid)
        except Exception as e:
            self.once_logger.warn("read_source_topic", f"Couldn't get data from {source_topic}")
            return

        # Create custom map by tracing the zones
        # TODO (Flor_Grosso): Create the rest of the zones here too.
        custom_map = self._add_no_go_zones_to(base_map)

        # Publish ROS Message if a valid map was created only
        # TODO (Flor_Grosso): send a notification to the client about this error.
        if custom_map is None:
            self.once_logger.warn(
                "map_design_error", "Map with spatial annotations couldn't be created."
            )
            return

        # Create the OccupancyGrid Message
        msg = nav_msgs.msg.OccupancyGrid()
        msg.header.stamp = self._ros.ros_now()
        msg.header.frame_id = base_map.header.frame_id
        # Preserve the map metadata from the base map (resolution,
        # width, height, origin)
        msg.info = base_map.info
        msg.data = custom_map

        # Publish the map
        self._ros_map_publisher.pub.publish(msg)
        self._costmap_published = True
        self.logger.info("Global map published.")

    def _add_no_go_zones_to(self, base_map):
        """
        Creates a customized map/costmap with no go zones in it, parting from the
        base_map.
        """

        # If there is no base map, abort the design.
        # NOTE (Flor_Grosso): consider supporting creating maps from scratch,
        # with no base and just the polygons.
        if not base_map:
            return None

        annotations = self._states["spatial_annotations"].get("annotations", [])

        # Get contours for no go zones only. This will return a list of objects
        # with 'data' (coordinates) and 'cost' fields.
        contours = filter(lambda x: x["type"] == "NO_GO_ZONE", annotations)
        # If there are no no go zones on the spatial annotations, return the
        # original map.
        if not contours:
            self.logger.warning(
                "No polygon coordinates received. Output map " "will be the same as input map."
            )
            return base_map.data

        # If there is a valid source map, use it as a base to create the
        # new one.
        map_width = base_map.info.width
        map_height = base_map.info.height

        # The data from ros comes as a 1d array, but Image
        # needs it as a 2d array.
        base_map_2d = np.reshape(base_map.data, (map_width, map_height))
        im = Image()
        updated_map_2d = im.fillPolygons(base_map_2d, contours, NO_GO_ZONE_COST)

        return updated_map_2d.flatten()

    def _is_source_map_current(self):
        """
        Checks if source data is current, by comparing the map checksum with the
        checksum of the data provided by the localizationAgentlet.
        """

        source_map_checksum = self._states["spatial_annotations"].get("map", {}).get("checksum")

        return (
            source_map_checksum is not None
            and source_map_checksum == self._localization.get_map_checksum()
        )

#!/usr/bin/env python3

import ctypes
import numpy as np
import cv2
from multiprocessing import shared_memory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


# Inline shared memory header (matches tools/shared_memory_utils.py)
class _SimpleImageHeader(ctypes.LittleEndianStructure):
    _fields_ = [
        ('timestamp', ctypes.c_uint64),
        ('height', ctypes.c_uint32),
        ('width', ctypes.c_uint32),
        ('channels', ctypes.c_uint32),
        ('image_name', ctypes.c_char * 16),
        ('data_size', ctypes.c_uint32),
        ('encoding', ctypes.c_uint32),
        ('quality', ctypes.c_uint32),
    ]


HEADER_SIZE = ctypes.sizeof(_SimpleImageHeader)


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')

        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('camera_name', 'head')
        self.declare_parameter('frame_id', 'front_cam')

        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, '/camera/image', 10)

        self.shm = None
        self.last_timestamp = 0

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"CameraBridge started: reading '{self.camera_name}' from shared memory, "
            f"publishing to /camera/image at {publish_rate} Hz"
        )

    def _read_image(self):
        """Read image from shared memory. Returns numpy BGR image or None."""
        shm_name = f"isaac_{self.camera_name}_image_shm"

        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=shm_name)
            except FileNotFoundError:
                return None

        header_data = bytes(self.shm.buf[:HEADER_SIZE])
        header = _SimpleImageHeader.from_buffer_copy(header_data)

        if header.timestamp <= self.last_timestamp:
            return None

        payload = bytes(self.shm.buf[HEADER_SIZE:HEADER_SIZE + header.data_size])

        if header.encoding == 1:  # JPEG
            encoded = np.frombuffer(payload, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        else:  # RAW
            image = np.frombuffer(payload, dtype=np.uint8)
            expected = header.height * header.width * header.channels
            if image.size != expected:
                return None
            image = image.reshape(header.height, header.width, header.channels)

        self.last_timestamp = header.timestamp
        return image

    def timer_callback(self):
        image = self._read_image()
        if image is None:
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = image.shape[1] * image.shape[2]
        msg.data = image.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.shm is not None:
            self.shm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

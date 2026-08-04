"""Small OpenCV to sensor_msgs/Image conversions without cv_bridge."""

import cv2
import numpy as np
from sensor_msgs.msg import Image


def image_message_to_bgr(message):
    """Return a BGR NumPy view copied from a bgr8 ROS image message."""
    if message.encoding not in ('bgr8', 'rgb8'):
        raise ValueError(
            f"Expected bgr8 or rgb8 image, received '{message.encoding}'."
        )
    channels = 3
    minimum_step = int(message.width) * channels
    if int(message.step) < minimum_step:
        raise ValueError(
            f'Image step {message.step} is smaller than {minimum_step}.'
        )

    rows = np.frombuffer(message.data, dtype=np.uint8)
    expected_size = int(message.height) * int(message.step)
    if rows.size < expected_size:
        raise ValueError(
            f'Image data has {rows.size} bytes; expected at least '
            f'{expected_size}.'
        )

    rows = rows[:expected_size].reshape(
        (int(message.height), int(message.step))
    )
    image = rows[:, :minimum_step].reshape(
        (int(message.height), int(message.width), channels)
    ).copy()
    if message.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def image_message_to_mono8(message):
    """Return a copied 2-D array from a mono8 ROS image message."""
    if message.encoding != 'mono8':
        raise ValueError(
            f"Expected mono8 image, received '{message.encoding}'."
        )
    minimum_step = int(message.width)
    if int(message.step) < minimum_step:
        raise ValueError(
            f'Image step {message.step} is smaller than {minimum_step}.'
        )

    rows = np.frombuffer(message.data, dtype=np.uint8)
    expected_size = int(message.height) * int(message.step)
    if rows.size < expected_size:
        raise ValueError(
            f'Image data has {rows.size} bytes; expected at least '
            f'{expected_size}.'
        )

    rows = rows[:expected_size].reshape(
        (int(message.height), int(message.step))
    )
    return rows[:, :minimum_step].copy()


def bgr_to_image_message(image, header):
    """Create a bgr8 sensor_msgs/Image from an OpenCV array."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected an HxWx3 BGR image.')
    contiguous = np.ascontiguousarray(image, dtype=np.uint8)
    message = Image()
    message.header = header
    message.height = int(contiguous.shape[0])
    message.width = int(contiguous.shape[1])
    message.encoding = 'bgr8'
    message.is_bigendian = 0
    message.step = int(contiguous.shape[1] * 3)
    message.data = contiguous.tobytes()
    return message


def mono8_to_image_message(image, header):
    """Create a mono8 sensor_msgs/Image from a 2-D uint8 array."""
    if image.ndim != 2:
        raise ValueError('Expected an HxW mono image.')
    contiguous = np.ascontiguousarray(image, dtype=np.uint8)
    message = Image()
    message.header = header
    message.height = int(contiguous.shape[0])
    message.width = int(contiguous.shape[1])
    message.encoding = 'mono8'
    message.is_bigendian = 0
    message.step = int(contiguous.shape[1])
    message.data = contiguous.tobytes()
    return message

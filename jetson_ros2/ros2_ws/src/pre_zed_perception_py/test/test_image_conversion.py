"""Unit tests for ROS image conversion helpers."""

import numpy as np
import pytest
from sensor_msgs.msg import Image

from pre_zed_perception_py.image_conversion import image_message_to_mono8


def make_mono8_message(width, height, step, data):
    message = Image()
    message.width = width
    message.height = height
    message.encoding = 'mono8'
    message.step = step
    message.data = bytes(data)
    return message


def test_image_message_to_mono8_ignores_row_padding():
    message = make_mono8_message(
        width=2,
        height=2,
        step=3,
        data=[1, 2, 99, 3, 4, 99],
    )

    image = image_message_to_mono8(message)

    np.testing.assert_array_equal(image, np.array([[1, 2], [3, 4]]))


def test_image_message_to_mono8_rejects_wrong_encoding():
    message = make_mono8_message(1, 1, 1, [1])
    message.encoding = 'bgr8'

    with pytest.raises(ValueError, match='Expected mono8'):
        image_message_to_mono8(message)


def test_image_message_to_mono8_rejects_short_data():
    message = make_mono8_message(2, 2, 2, [1, 2])

    with pytest.raises(ValueError, match='expected at least 4'):
        image_message_to_mono8(message)

"""Unit tests for the temporary JSON contract."""

import json

from pre_zed_perception_py.contracts import compact_json, detection_message


def test_detection_message_preserves_numeric_types():
    payload = detection_message(
        stamp_ns=123,
        frame_id='camera',
        width=640,
        height=480,
        model_id='example/1',
        detections=[],
        inference_ms=10.25,
    )
    decoded = json.loads(compact_json(payload))
    assert decoded['stamp_ns'] == 123
    assert decoded['image_width'] == 640
    assert decoded['inference_ms'] == 10.25

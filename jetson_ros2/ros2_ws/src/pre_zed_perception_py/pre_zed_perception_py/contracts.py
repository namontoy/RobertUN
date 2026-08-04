"""Helpers for the temporary JSON topic contracts used by the pre-ZED demo."""

import json


SCHEMA_VERSION = '0.1.0'


def stamp_to_nanoseconds(stamp):
    """Convert a ROS builtin_interfaces/Time-like object to nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def compact_json(value):
    """Serialize one topic payload with deterministic compact formatting."""
    return json.dumps(value, separators=(',', ':'), sort_keys=True)


def detection_message(stamp_ns, frame_id, width, height, model_id, detections,
                      inference_ms):
    """Build the detection JSON payload shared with the C++ tracker."""
    return {
        'schema_version': SCHEMA_VERSION,
        'stamp_ns': int(stamp_ns),
        'frame_id': str(frame_id),
        'image_width': int(width),
        'image_height': int(height),
        'model_id': str(model_id),
        'inference_ms': round(float(inference_ms), 3),
        'detections': detections,
    }

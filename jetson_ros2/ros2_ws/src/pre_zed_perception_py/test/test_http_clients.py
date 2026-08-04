"""Unit tests for inference response conversion."""

import numpy as np
import pytest

from pre_zed_perception_py.http_clients import (
    InferenceError,
    normalize_depth_to_mono8,
    parse_detection_response,
)


def test_parse_detection_response_converts_and_clips_bbox():
    response = {
        'predictions': [{
            'x': 10,
            'y': 10,
            'width': 30,
            'height': 20,
            'confidence': 0.8,
            'class': 'car',
            'class_id': 2,
        }]
    }
    detections = parse_detection_response(response, 0.5, 100, 80)
    assert detections == [{
        'label': 'car',
        'class_id': 2,
        'confidence': 0.8,
        'bbox_xywh': [0.0, 0.0, 25.0, 20.0],
    }]


def test_parse_detection_response_filters_confidence_and_class():
    response = {
        'predictions': [
            {
                'x': 50,
                'y': 40,
                'width': 20,
                'height': 20,
                'confidence': 0.2,
                'class': 'car',
            },
            {
                'x': 50,
                'y': 40,
                'width': 20,
                'height': 20,
                'confidence': 0.9,
                'class': 'person',
            },
        ]
    }
    detections = parse_detection_response(
        response, 0.5, 100, 80, allowed_classes=['car']
    )
    assert detections == []


def test_parse_detection_response_rejects_missing_predictions():
    with pytest.raises(InferenceError):
        parse_detection_response({}, 0.5, 100, 80)


def test_parse_detection_response_tolerates_invalid_class_id():
    response = {
        'predictions': [{
            'x': 50,
            'y': 40,
            'width': 20,
            'height': 20,
            'confidence': 0.9,
            'class': 'car',
            'class_id': None,
        }]
    }
    detections = parse_detection_response(response, 0.5, 100, 80)
    assert detections[0]['class_id'] == -1


def test_normalize_depth_resizes_and_handles_non_finite_values():
    depth = np.array([[0.0, 1.0], [np.nan, 2.0]], dtype=np.float32)
    result = normalize_depth_to_mono8(depth, 4, 4)
    assert result.shape == (4, 4)
    assert result.dtype == np.uint8
    assert int(result.max()) == 255


def test_normalize_depth_rejects_non_image_shape():
    with pytest.raises(InferenceError):
        normalize_depth_to_mono8(np.zeros((2, 2, 2)), 2, 2)

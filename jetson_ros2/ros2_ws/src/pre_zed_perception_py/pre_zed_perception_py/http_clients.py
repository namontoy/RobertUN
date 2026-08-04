"""HTTP clients and response validation for Roboflow Inference."""

import base64

import cv2
import numpy as np
import requests


class InferenceError(RuntimeError):
    """Raised when an inference request or response is invalid."""


def _encode_jpeg_base64(image_bgr, jpeg_quality):
    ok, encoded = cv2.imencode(
        '.jpg',
        image_bgr,
        [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)],
    )
    if not ok:
        raise InferenceError('Could not encode the ROS image as JPEG.')
    return base64.b64encode(encoded.tobytes()).decode('ascii')


def _response_json(response, task_name):
    if response.status_code >= 400:
        raise InferenceError(
            f'{task_name} HTTP request failed with status '
            f'{response.status_code}: {response.text[:500]}'
        )
    try:
        return response.json()
    except requests.exceptions.JSONDecodeError as exc:
        raise InferenceError(
            f'{task_name} response was not valid JSON: {response.text[:500]}'
        ) from exc


def normalize_depth_to_mono8(depth, output_width, output_height):
    """Normalize one relative-depth frame to mono8 for transport and fusion."""
    array = np.asarray(depth, dtype=np.float32)
    array = np.squeeze(array)
    if array.ndim != 2:
        raise InferenceError(
            f'Expected a two-dimensional depth map, received shape {array.shape}.'
        )

    finite = np.isfinite(array)
    if not finite.any():
        raise InferenceError('Depth response contains no finite values.')

    valid = array[finite]
    minimum = float(valid.min())
    maximum = float(valid.max())
    normalized = np.zeros(array.shape, dtype=np.float32)
    if maximum > minimum:
        normalized[finite] = (array[finite] - minimum) / (maximum - minimum)

    # Reserve zero for invalid samples so the C++ fusion node can exclude them.
    mono8 = np.zeros(array.shape, dtype=np.uint8)
    mono8[finite] = np.clip(
        normalized[finite] * 254.0 + 1.0, 1.0, 255.0
    ).astype(np.uint8)
    if mono8.shape != (int(output_height), int(output_width)):
        mono8 = cv2.resize(
            mono8,
            (int(output_width), int(output_height)),
            interpolation=cv2.INTER_LINEAR,
        )
    return mono8


def parse_detection_response(data, confidence_threshold, image_width,
                             image_height, allowed_classes=None):
    """Convert Roboflow center-based predictions to clipped xywh boxes."""
    if isinstance(data, list):
        if len(data) == 1 and isinstance(data[0], dict):
            data = data[0]
        else:
            raise InferenceError('Detection response list has an unknown shape.')
    if not isinstance(data, dict):
        raise InferenceError('Detection response must be a JSON object.')

    predictions = data.get('predictions')
    if not isinstance(predictions, list):
        raise InferenceError("Detection response is missing a 'predictions' list.")

    allowed = set(allowed_classes or [])
    detections = []
    for prediction in predictions:
        if not isinstance(prediction, dict):
            continue
        try:
            center_x = float(prediction['x'])
            center_y = float(prediction['y'])
            box_width = float(prediction['width'])
            box_height = float(prediction['height'])
            confidence = float(prediction['confidence'])
        except (KeyError, TypeError, ValueError):
            continue

        label = str(
            prediction.get('class', prediction.get('class_name', 'unknown'))
        )
        if confidence < float(confidence_threshold):
            continue
        if allowed and label not in allowed:
            continue
        if box_width <= 0.0 or box_height <= 0.0:
            continue

        left = max(0.0, center_x - box_width / 2.0)
        top = max(0.0, center_y - box_height / 2.0)
        right = min(float(image_width), center_x + box_width / 2.0)
        bottom = min(float(image_height), center_y + box_height / 2.0)
        if right <= left or bottom <= top:
            continue

        try:
            class_id = int(prediction.get('class_id', -1))
        except (TypeError, ValueError):
            class_id = -1
        detections.append({
            'label': label,
            'class_id': class_id,
            'confidence': round(confidence, 6),
            'bbox_xywh': [
                round(left, 3),
                round(top, 3),
                round(right - left, 3),
                round(bottom - top, 3),
            ],
        })
    return detections


class DepthAnythingHttpClient:
    """Client for the Depth Anything endpoint already used by this repository."""

    def __init__(self, server_url, model_id, api_key='', timeout_seconds=120.0,
                 jpeg_quality=90, session=None):
        self.endpoint = f"{server_url.rstrip('/')}/infer/depth-estimation"
        self.model_id = model_id
        self.api_key = api_key
        self.timeout_seconds = float(timeout_seconds)
        self.jpeg_quality = int(jpeg_quality)
        self.session = session or requests.Session()

    def infer(self, image_bgr, request_id):
        image_base64 = _encode_jpeg_base64(image_bgr, self.jpeg_quality)
        payload = {
            'id': str(request_id),
            'model_id': self.model_id,
            'image': {'type': 'base64', 'value': image_base64},
            'depth_version_id': self.model_id.rstrip('/').split('/')[-1],
        }
        params = {'api_key': self.api_key} if self.api_key else None
        try:
            response = self.session.post(
                self.endpoint,
                json=payload,
                params=params,
                timeout=self.timeout_seconds,
            )
        except requests.RequestException as exc:
            raise InferenceError(f'Depth HTTP request failed: {exc}') from exc
        data = _response_json(response, 'Depth')
        if not isinstance(data, dict) or 'normalized_depth' not in data:
            keys = list(data.keys()) if isinstance(data, dict) else type(data)
            raise InferenceError(
                f"Depth response missing 'normalized_depth'; received {keys}."
            )
        return np.asarray(data['normalized_depth'], dtype=np.float32)


class YoloHttpClient:
    """Client for Roboflow's generic object-detection endpoint."""

    def __init__(self, server_url, model_id, api_key='', timeout_seconds=120.0,
                 jpeg_quality=90, confidence_threshold=0.35,
                 iou_threshold=0.5, session=None):
        self.endpoint = f"{server_url.rstrip('/')}/infer/object_detection"
        self.model_id = model_id
        self.api_key = api_key
        self.timeout_seconds = float(timeout_seconds)
        self.jpeg_quality = int(jpeg_quality)
        self.confidence_threshold = float(confidence_threshold)
        self.iou_threshold = float(iou_threshold)
        self.session = session or requests.Session()

    def infer(self, image_bgr, request_id, allowed_classes=None):
        image_base64 = _encode_jpeg_base64(image_bgr, self.jpeg_quality)
        payload = {
            'id': str(request_id),
            'model_id': self.model_id,
            'image': {'type': 'base64', 'value': image_base64},
            'confidence': self.confidence_threshold,
            'iou_threshold': self.iou_threshold,
        }
        if self.api_key:
            payload['api_key'] = self.api_key
        try:
            response = self.session.post(
                self.endpoint,
                json=payload,
                timeout=self.timeout_seconds,
            )
        except requests.RequestException as exc:
            raise InferenceError(f'Detection HTTP request failed: {exc}') from exc
        data = _response_json(response, 'Detection')
        return parse_detection_response(
            data,
            confidence_threshold=self.confidence_threshold,
            image_width=image_bgr.shape[1],
            image_height=image_bgr.shape[0],
            allowed_classes=allowed_classes,
        )

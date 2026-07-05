#!/usr/bin/env python3

import argparse
import base64
import csv
import os
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import requests


DEFAULT_MODEL_ID = 'depth-anything-v3/small'
DEFAULT_INPUT_DIR = 'datasets/depth_anything_v3_small/input_videos'
DEFAULT_OUTPUT_DIR = 'datasets/depth_anything_v3_small/output_videos'
DEFAULT_REPORT_DIR = 'benchmarks/depth_anything_v3_small'
DEFAULT_SERVER_URL = 'http://localhost:9001'


@dataclass
class VideoMetrics:
    video_name: str
    frame_count: int
    processed_frames: int
    total_seconds: float
    avg_fps: float
    min_fps: float
    max_fps: float
    avg_inference_ms: float
    output_video_path: str


class TegraStatsSampler:
    def __init__(self, enabled=True, interval_ms=1000):
        self.enabled = enabled
        self.interval_ms = interval_ms
        self.process = None
        self.lines = []

    def __enter__(self):
        if not self.enabled:
            return self

        if not _command_exists('tegrastats'):
            self.lines.append('tegrastats unavailable')
            return self

        self.process = subprocess.Popen(
            ['tegrastats', '--interval', str(self.interval_ms)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        if self.process is None:
            return

        self.process.terminate()
        try:
            stdout, _ = self.process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            stdout, _ = self.process.communicate()

        self.lines.extend(line for line in stdout.splitlines() if line.strip())


class NativeRoboflowDepthModel:
    def __init__(self, model_id):
        try:
            from inference import get_model
        except ImportError as exc:
            raise RuntimeError(
                "Roboflow local runtime is not installed. Expected Python package "
                "'inference' with 'from inference import get_model'."
            ) from exc

        self.model_id = model_id
        self.model = get_model(model_id=model_id)

    def infer_depth(self, frame_bgr):
        prediction = self.model.infer(frame_bgr)
        return _prediction_to_depth_array(prediction)


class HttpRoboflowDepthModel:
    def __init__(self, model_id, server_url, api_key=''):
        self.model_id = model_id
        self.server_url = server_url.rstrip('/')
        self.endpoint = f'{self.server_url}/infer/depth-estimation'
        self.depth_version_id = _depth_version_from_model_id(model_id)
        self.api_key = api_key

    def infer_depth(self, frame_bgr):
        ok, encoded = cv2.imencode('.jpg', frame_bgr)
        if not ok:
            raise RuntimeError('Could not encode frame as JPEG for HTTP inference.')

        image_base64 = base64.b64encode(encoded.tobytes()).decode('ascii')
        payload = {
            'id': 'video-frame',
            'model_id': self.model_id,
            'image': {
                'type': 'base64',
                'value': image_base64,
            },
            'depth_version_id': self.depth_version_id,
        }

        params = {'api_key': self.api_key} if self.api_key else None
        response = requests.post(self.endpoint, json=payload, params=params, timeout=120)
        if response.status_code >= 400:
            raise RuntimeError(
                f'HTTP depth inference failed: {response.status_code} {response.text[:500]}'
            )

        data = response.json()
        if 'normalized_depth' not in data:
            raise RuntimeError(f"Depth response missing 'normalized_depth': {data.keys()}")
        return np.asarray(data['normalized_depth'], dtype=np.float32)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Benchmark Depth Anything V3 Small on local MP4 videos.'
    )
    parser.add_argument('--model-id', default=DEFAULT_MODEL_ID)
    parser.add_argument('--input-dir', default=DEFAULT_INPUT_DIR)
    parser.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    parser.add_argument('--report-dir', default=DEFAULT_REPORT_DIR)
    parser.add_argument('--backend', choices=('http', 'native'), default='http')
    parser.add_argument('--server-url', default=DEFAULT_SERVER_URL)
    parser.add_argument('--api-key', default=os.environ.get('ROBOFLOW_API_KEY', ''))
    parser.add_argument('--max-videos', type=int, default=10)
    parser.add_argument('--frame-stride', type=int, default=1)
    parser.add_argument('--no-tegrastats', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    report_dir = Path(args.report_dir)

    videos = sorted(input_dir.glob('*.mp4'))[:args.max_videos]
    if not videos:
        _ensure_dirs(input_dir, output_dir, report_dir)
        _write_empty_reports(report_dir, input_dir)
        print(f'No MP4 files found in {input_dir}')
        print('Add videos there and rerun the benchmark.')
        return 0

    output_dir.mkdir(parents=True, exist_ok=True)
    report_dir.mkdir(parents=True, exist_ok=True)

    print(f'Loading model: {args.model_id} with backend={args.backend}')
    model = _create_depth_model(args)

    all_metrics = []
    all_stats_rows = []

    for video_path in videos:
        print(f'Processing {video_path.name}')
        with TegraStatsSampler(enabled=not args.no_tegrastats) as tegrastats:
            metrics = process_video(
                video_path=video_path,
                output_dir=output_dir,
                model=model,
                frame_stride=max(1, args.frame_stride),
            )

        all_metrics.append(metrics)
        all_stats_rows.extend(_stats_rows(video_path.name, tegrastats.lines))

    _write_fps_table(report_dir / 'fps_table.csv', all_metrics)
    _write_gpu_table(report_dir / 'gpu_memory_table.csv', all_stats_rows)
    _write_quality_notes(report_dir / 'quality_notes.md', all_metrics)
    _write_summary(report_dir / 'summary.md', args.model_id, all_metrics)

    print(f'Wrote reports to {report_dir}')
    return 0


def _create_depth_model(args):
    if args.backend == 'http':
        return HttpRoboflowDepthModel(args.model_id, args.server_url, args.api_key)
    return NativeRoboflowDepthModel(args.model_id)


def _depth_version_from_model_id(model_id):
    version = str(model_id).rstrip('/').split('/')[-1]
    return version or 'small'


def process_video(video_path, output_dir, model, frame_stride):
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f'Could not open video: {video_path}')

    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    source_fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    output_path = output_dir / f'{video_path.stem}_depth_viz.mp4'
    writer = _create_video_writer(output_path, source_fps / frame_stride, width, height)

    processed_frames = 0
    frame_index = 0
    frame_fps_values = []
    inference_ms_values = []
    started_at = time.perf_counter()

    while True:
        ok, frame = capture.read()
        if not ok:
            break

        if frame_index % frame_stride != 0:
            frame_index += 1
            continue

        frame_started = time.perf_counter()
        depth = model.infer_depth(frame)
        inference_done = time.perf_counter()

        depth_viz = _depth_to_bgr_visualization(depth, width, height)
        writer.write(depth_viz)

        frame_elapsed = time.perf_counter() - frame_started
        inference_ms_values.append((inference_done - frame_started) * 1000.0)
        if frame_elapsed > 0:
            frame_fps_values.append(1.0 / frame_elapsed)

        processed_frames += 1
        frame_index += 1

    total_seconds = time.perf_counter() - started_at
    capture.release()
    writer.release()

    avg_fps = processed_frames / total_seconds if total_seconds > 0 else 0.0
    return VideoMetrics(
        video_name=video_path.name,
        frame_count=frame_count,
        processed_frames=processed_frames,
        total_seconds=total_seconds,
        avg_fps=avg_fps,
        min_fps=min(frame_fps_values, default=0.0),
        max_fps=max(frame_fps_values, default=0.0),
        avg_inference_ms=float(np.mean(inference_ms_values)) if inference_ms_values else 0.0,
        output_video_path=str(output_path),
    )


def _prediction_to_depth_array(prediction):
    candidate = _unwrap_prediction(prediction)

    if isinstance(candidate, np.ndarray):
        return candidate

    if hasattr(candidate, 'numpy'):
        return candidate.numpy()

    if isinstance(candidate, dict):
        for key in ('depth', 'depth_map', 'prediction', 'predictions', 'image'):
            if key in candidate:
                return _prediction_to_depth_array(candidate[key])

    if hasattr(candidate, 'depth'):
        return _prediction_to_depth_array(candidate.depth)
    if hasattr(candidate, 'predictions'):
        return _prediction_to_depth_array(candidate.predictions)
    if hasattr(candidate, 'image'):
        return _prediction_to_depth_array(candidate.image)

    raise RuntimeError(
        f'Could not convert Roboflow prediction to a depth array: {type(candidate)!r}'
    )


def _unwrap_prediction(prediction):
    if isinstance(prediction, (list, tuple)):
        if not prediction:
            raise RuntimeError('Roboflow prediction result was empty.')
        return prediction[0]
    return prediction


def _depth_to_bgr_visualization(depth, width, height):
    depth_array = np.asarray(depth)

    if depth_array.ndim == 3 and depth_array.shape[2] == 3:
        viz = depth_array
        if viz.dtype != np.uint8:
            viz = _normalize_to_uint8(viz)
        if viz.shape[1] != width or viz.shape[0] != height:
            viz = cv2.resize(viz, (width, height), interpolation=cv2.INTER_LINEAR)
        return viz

    if depth_array.ndim == 3 and depth_array.shape[0] == 1:
        depth_array = depth_array[0]
    if depth_array.ndim == 3 and depth_array.shape[2] == 1:
        depth_array = depth_array[:, :, 0]

    depth_u8 = _normalize_to_uint8(depth_array)
    if depth_u8.shape[1] != width or depth_u8.shape[0] != height:
        depth_u8 = cv2.resize(depth_u8, (width, height), interpolation=cv2.INTER_LINEAR)
    return cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)


def _normalize_to_uint8(array):
    array = np.asarray(array, dtype=np.float32)
    finite = np.isfinite(array)
    if not finite.any():
        return np.zeros(array.shape[:2], dtype=np.uint8)

    valid = array[finite]
    min_value = float(valid.min())
    max_value = float(valid.max())
    if max_value <= min_value:
        return np.zeros(array.shape[:2], dtype=np.uint8)

    normalized = (array - min_value) / (max_value - min_value)
    normalized = np.clip(normalized * 255.0, 0, 255)
    return normalized.astype(np.uint8)


def _create_video_writer(output_path, fps, width, height):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(str(output_path), fourcc, max(1.0, fps), (width, height))
    if not writer.isOpened():
        raise RuntimeError(f'Could not create output video: {output_path}')
    return writer


def _write_fps_table(path, metrics):
    with path.open('w', newline='') as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                'video_name',
                'frame_count',
                'processed_frames',
                'total_seconds',
                'avg_fps',
                'min_fps',
                'max_fps',
                'avg_inference_ms',
                'output_video_path',
            ],
        )
        writer.writeheader()
        for row in metrics:
            writer.writerow({
                'video_name': row.video_name,
                'frame_count': row.frame_count,
                'processed_frames': row.processed_frames,
                'total_seconds': f'{row.total_seconds:.3f}',
                'avg_fps': f'{row.avg_fps:.3f}',
                'min_fps': f'{row.min_fps:.3f}',
                'max_fps': f'{row.max_fps:.3f}',
                'avg_inference_ms': f'{row.avg_inference_ms:.3f}',
                'output_video_path': row.output_video_path,
            })


def _write_gpu_table(path, rows):
    with path.open('w', newline='') as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                'video_name',
                'sample_index',
                'ram_used_mb',
                'ram_total_mb',
                'gpu_load_percent',
                'raw_tegrastats_line',
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _write_quality_notes(path, metrics):
    with path.open('w') as file:
        file.write('# Depth Anything V3 Small Quality Notes\n\n')
        for metric in metrics:
            file.write(f'## {metric.video_name}\n\n')
            file.write('- useful_depth: yes/no/maybe\n')
            file.write('- flicker: low/medium/high\n')
            file.write('- edge_quality: low/medium/high\n')
            file.write('- relative_depth_consistency: low/medium/high\n')
            file.write('- notes:\n\n')


def _write_summary(path, model_id, metrics):
    avg_fps_values = [metric.avg_fps for metric in metrics]
    average_fps = float(np.mean(avg_fps_values)) if avg_fps_values else 0.0
    with path.open('w') as file:
        file.write('# Depth Anything V3 Small Benchmark Summary\n\n')
        file.write(f'- model_id: `{model_id}`\n')
        file.write(f'- videos_processed: {len(metrics)}\n')
        file.write(f'- average_fps_across_videos: {average_fps:.3f}\n')
        file.write('- decision: continue/pause/replace\n\n')
        file.write('## Notes\n\n')
        file.write('Depth output is relative monocular depth, not metric distance in meters.\n')


def _write_empty_reports(report_dir, input_dir):
    report_dir.mkdir(parents=True, exist_ok=True)
    _write_fps_table(report_dir / 'fps_table.csv', [])
    _write_gpu_table(report_dir / 'gpu_memory_table.csv', [{
        'video_name': '',
        'sample_index': 0,
        'ram_used_mb': '',
        'ram_total_mb': '',
        'gpu_load_percent': '',
        'raw_tegrastats_line': f'No MP4 files found in {input_dir}',
    }])
    _write_quality_notes(report_dir / 'quality_notes.md', [])
    _write_summary(report_dir / 'summary.md', DEFAULT_MODEL_ID, [])


def _stats_rows(video_name, lines):
    if not lines:
        return [{
            'video_name': video_name,
            'sample_index': 0,
            'ram_used_mb': '',
            'ram_total_mb': '',
            'gpu_load_percent': '',
            'raw_tegrastats_line': 'no tegrastats samples captured',
        }]

    rows = []
    for index, line in enumerate(lines):
        rows.append({
            'video_name': video_name,
            'sample_index': index,
            'ram_used_mb': _parse_ram(line)[0],
            'ram_total_mb': _parse_ram(line)[1],
            'gpu_load_percent': _parse_gpu(line),
            'raw_tegrastats_line': line,
        })
    return rows


def _parse_ram(line):
    match = re.search(r'RAM\s+(\d+)/(\d+)MB', line)
    if not match:
        return '', ''
    return match.group(1), match.group(2)


def _parse_gpu(line):
    match = re.search(r'GR3D_FREQ\s+(\d+)%', line)
    return match.group(1) if match else ''


def _ensure_dirs(*paths):
    for path in paths:
        Path(path).mkdir(parents=True, exist_ok=True)


def _command_exists(command):
    return subprocess.run(
        ['which', command],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    ).returncode == 0


if __name__ == '__main__':
    raise SystemExit(main())

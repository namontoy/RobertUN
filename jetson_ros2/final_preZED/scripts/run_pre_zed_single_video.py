#!/usr/bin/env python3
"""Run and validate one finite pre-ZED video through the ROS 2 pipeline."""

import argparse
import math
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

import cv2


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VIDEO = (
    REPO_ROOT
    / 'datasets'
    / 'depth_anything_v3_small'
    / 'input_videos'
    / 'humanoidrobot.mp4'
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'final_preZED' / 'outputs'
CONFIG_FILE = REPO_ROOT / 'final_preZED' / 'configs' / 'pre_zed_params.yaml'


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Process one local video sequentially through detection, '
            'tracking, '
            'relative depth, fusion, and overlay recording.'
        )
    )
    parser.add_argument(
        '--video',
        type=Path,
        default=DEFAULT_VIDEO,
        help='Input MP4. Defaults to the shortest dataset video.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        help=(
            'Output MP4. Defaults to '
            'final_preZED/outputs/<input>_pre_zed_overlay.mp4.'
        ),
    )
    parser.add_argument(
        '--frame-stride',
        type=int,
        default=1,
        help='Process every Nth source frame. Use 1 for a complete video.',
    )
    parser.add_argument(
        '--completion-timeout',
        type=float,
        default=150.0,
        help='Maximum seconds for one frame to reach the overlay.',
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Replace an existing output file.',
    )
    return parser.parse_args()


def read_video_metadata(path):
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f'OpenCV could not open video: {path}')

    metadata = {
        'frame_count': int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
        'fps': float(capture.get(cv2.CAP_PROP_FPS)),
        'width': int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        'height': int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    }
    capture.release()
    if (
        metadata['frame_count'] <= 0
        or metadata['fps'] <= 0.0
        or metadata['width'] <= 0
        or metadata['height'] <= 0
    ):
        raise RuntimeError(f'Invalid video metadata for: {path}')
    return metadata


def inspect_output(path):
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f'OpenCV could not decode output: {path}')
    metadata = {
        'frame_count': int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
        'fps': float(capture.get(cv2.CAP_PROP_FPS)),
        'width': int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        'height': int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    }
    first_frame_ok, _ = capture.read()
    capture.release()
    metadata['first_frame_ok'] = first_frame_ok
    return metadata


def validate_output(source, output, frame_stride):
    expected_frames = math.ceil(source['frame_count'] / frame_stride)
    decode_tolerance = max(10, math.ceil(expected_frames * 0.03))
    minimum_frames = max(1, expected_frames - decode_tolerance)
    expected_fps = source['fps'] / frame_stride

    errors = []
    if not output['first_frame_ok']:
        errors.append('the first output frame cannot be decoded')
    if output['frame_count'] < minimum_frames:
        errors.append(
            f"only {output['frame_count']} frames were written; expected "
            f'at least {minimum_frames}'
        )
    if output['width'] != source['width']:
        errors.append(
            f"output width {output['width']} != source width {source['width']}"
        )
    if output['height'] != source['height']:
        errors.append(
            f"output height {output['height']} != source height "
            f"{source['height']}"
        )
    if abs(output['fps'] - expected_fps) > 0.1:
        errors.append(
            f"output FPS {output['fps']:.3f} != expected "
            f'{expected_fps:.3f}'
        )
    return errors


def resolve_path(path):
    path = path.expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve()


def main():
    args = parse_args()
    if args.frame_stride < 1:
        print('error: --frame-stride must be at least 1', file=sys.stderr)
        return 2
    if args.completion_timeout <= 0.0:
        print(
            'error: --completion-timeout must be greater than zero',
            file=sys.stderr,
        )
        return 2

    video_path = resolve_path(args.video)
    if not video_path.is_file():
        print(
            f'error: input video does not exist: {video_path}',
            file=sys.stderr,
        )
        return 2

    output_path = args.output
    if output_path is None:
        output_path = (
            DEFAULT_OUTPUT_DIR
            / f'{video_path.stem}_pre_zed_overlay.mp4'
        )
    output_path = resolve_path(output_path)
    if output_path.suffix.lower() != '.mp4':
        print('error: --output must use the .mp4 extension', file=sys.stderr)
        return 2
    if output_path.exists() and not args.overwrite:
        print(
            f'error: output already exists: {output_path}\n'
            'Use --overwrite only when replacing it intentionally.',
            file=sys.stderr,
        )
        return 2

    if shutil.which('ros2') is None:
        print(
            'error: ros2 is unavailable; source ROS 2 and the workspace first',
            file=sys.stderr,
        )
        return 2
    if not CONFIG_FILE.is_file():
        print(f'error: config file is missing: {CONFIG_FILE}', file=sys.stderr)
        return 2

    try:
        source = read_video_metadata(video_path)
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    if output_path.exists():
        output_path.unlink()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    output_fps = source['fps'] / args.frame_stride
    expected_frames = math.ceil(
        source['frame_count'] / args.frame_stride
    )
    estimated_minutes = expected_frames / 0.3 / 60.0
    print(
        f"Input: {video_path}\n"
        f"Metadata: {source['width']}x{source['height']}, "
        f"{source['frame_count']} frames, {source['fps']:.3f} FPS\n"
        f'Expected output: about {expected_frames} frames at '
        f'{output_fps:.3f} FPS\n'
        f'Conservative runtime estimate: {estimated_minutes:.1f} minutes\n'
        f'Output: {output_path}'
    )

    command = [
        'ros2',
        'launch',
        'pre_zed_perception_py',
        'pre_zed_pipeline.launch.py',
        f'config_file:={CONFIG_FILE}',
        f'video_path:={video_path}',
        f'output_video_path:={output_path}',
        f'output_video_fps:={output_fps}',
        'loop:=false',
        'sequential_mode:=true',
        f'frame_stride:={args.frame_stride}',
        f'completion_timeout_seconds:={args.completion_timeout}',
    ]
    print(f'Launching: {shlex.join(command)}')
    try:
        result = subprocess.run(command, cwd=REPO_ROOT, check=False)
    except KeyboardInterrupt:
        print('Interrupted; the output may be incomplete.', file=sys.stderr)
        return 130
    if result.returncode != 0:
        print(
            f'error: ROS launch exited with code {result.returncode}',
            file=sys.stderr,
        )
        return result.returncode
    if not output_path.is_file() or output_path.stat().st_size == 0:
        print(f'error: output was not created: {output_path}', file=sys.stderr)
        return 1

    try:
        output = inspect_output(output_path)
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    errors = validate_output(source, output, args.frame_stride)
    print(
        f"Output metadata: {output['width']}x{output['height']}, "
        f"{output['frame_count']} frames, {output['fps']:.3f} FPS"
    )
    if errors:
        for error in errors:
            print(f'validation error: {error}', file=sys.stderr)
        return 1

    duration = output['frame_count'] / output['fps']
    print(
        f'Validation passed: complete decodable MP4, '
        f'{duration:.2f} seconds.'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

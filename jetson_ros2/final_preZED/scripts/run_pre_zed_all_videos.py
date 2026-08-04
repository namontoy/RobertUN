#!/usr/bin/env python3
"""Process and validate every dataset video through the pre-ZED pipeline."""

import argparse
import math
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

from run_pre_zed_single_video import (
    CONFIG_FILE,
    DEFAULT_OUTPUT_DIR,
    inspect_output,
    read_video_metadata,
    resolve_path,
    validate_output,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT_DIR = (
    REPO_ROOT
    / 'datasets'
    / 'depth_anything_v3_small'
    / 'input_videos'
)
DEFAULT_BATCH_OUTPUT_DIR = DEFAULT_OUTPUT_DIR / 'all_videos'
SINGLE_VIDEO_RUNNER = Path(__file__).with_name(
    'run_pre_zed_single_video.py'
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Process each MP4 independently through detection, tracking, '
            'relative depth, fusion, and validated overlay recording.'
        )
    )
    parser.add_argument(
        '--input-dir',
        type=Path,
        default=DEFAULT_INPUT_DIR,
        help='Directory containing input MP4 files.',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=DEFAULT_BATCH_OUTPUT_DIR,
        help=(
            'Directory for independent overlay MP4 files. Defaults to '
            'final_preZED/outputs/all_videos.'
        ),
    )
    parser.add_argument(
        '--frame-stride',
        type=int,
        default=1,
        help='Process every Nth frame. The complete workflow uses 1.',
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
        help='Replace existing outputs, including valid outputs.',
    )
    parser.add_argument(
        '--continue-on-error',
        action='store_true',
        help='Continue with later videos after one video fails.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the ordered workload without launching ROS.',
    )
    return parser.parse_args()


def inspect_inputs(input_dir):
    videos = sorted(input_dir.glob('*.mp4'))
    inspected = []
    for video in videos:
        metadata = read_video_metadata(video)
        inspected.append((metadata['frame_count'], video, metadata))
    inspected.sort(key=lambda item: (item[0], item[1].name.lower()))
    return [(video, metadata) for _, video, metadata in inspected]


def valid_existing_output(output_path, source, frame_stride):
    try:
        output = inspect_output(output_path)
    except RuntimeError as exc:
        return False, [str(exc)]
    errors = validate_output(source, output, frame_stride)
    return not errors, errors


def output_path_for(video, output_dir):
    return output_dir / f'{video.stem}_pre_zed_overlay.mp4'


def print_workload(workload, output_dir, frame_stride):
    total_frames = 0
    print(f'Input videos: {len(workload)}')
    print(f'Output directory: {output_dir}')
    print(f'Frame stride: {frame_stride}')
    for index, (video, metadata) in enumerate(workload, start=1):
        frames = math.ceil(metadata['frame_count'] / frame_stride)
        total_frames += frames
        output = output_path_for(video, output_dir)
        print(
            f'  {index}. {video.name}: about {frames} output frames -> '
            f'{output.name}'
        )
    estimated_minutes = total_frames / 0.3 / 60.0
    print(
        f'Total: about {total_frames} frames; conservative runtime '
        f'estimate {estimated_minutes:.1f} minutes.'
    )


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

    input_dir = resolve_path(args.input_dir)
    output_dir = resolve_path(args.output_dir)
    if not input_dir.is_dir():
        print(
            f'error: input directory does not exist: {input_dir}',
            file=sys.stderr,
        )
        return 2
    try:
        workload = inspect_inputs(input_dir)
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if not workload:
        print(f'error: no MP4 files found in {input_dir}', file=sys.stderr)
        return 2

    print_workload(workload, output_dir, args.frame_stride)
    if args.dry_run:
        print('Dry run complete; ROS was not launched.')
        return 0

    if shutil.which('ros2') is None:
        print(
            'error: ros2 is unavailable; source ROS 2 and the workspace first',
            file=sys.stderr,
        )
        return 2
    if not CONFIG_FILE.is_file():
        print(f'error: config file is missing: {CONFIG_FILE}', file=sys.stderr)
        return 2
    if not SINGLE_VIDEO_RUNNER.is_file():
        print(
            f'error: single-video runner is missing: {SINGLE_VIDEO_RUNNER}',
            file=sys.stderr,
        )
        return 2

    output_dir.mkdir(parents=True, exist_ok=True)
    processed = []
    skipped = []
    failures = []

    for index, (video, source) in enumerate(workload, start=1):
        output = output_path_for(video, output_dir)
        print(
            f'\n=== Video {index}/{len(workload)}: {video.name} ===',
            flush=True,
        )
        if output.exists() and not args.overwrite:
            is_valid, errors = valid_existing_output(
                output, source, args.frame_stride
            )
            if is_valid:
                print(f'Skipping validated existing output: {output}')
                skipped.append(video.name)
                continue
            failures.append(video.name)
            print(
                f'error: existing output is invalid: {output}',
                file=sys.stderr,
            )
            for error in errors:
                print(f'  validation error: {error}', file=sys.stderr)
            print(
                'Rerun with --overwrite after reviewing that file.',
                file=sys.stderr,
            )
            if not args.continue_on_error:
                break
            continue

        command = [
            sys.executable,
            str(SINGLE_VIDEO_RUNNER),
            '--video',
            str(video),
            '--output',
            str(output),
            '--frame-stride',
            str(args.frame_stride),
            '--completion-timeout',
            str(args.completion_timeout),
        ]
        if args.overwrite:
            command.append('--overwrite')
        print(f'Running: {shlex.join(command)}', flush=True)
        result = subprocess.run(command, cwd=REPO_ROOT, check=False)
        if result.returncode == 0:
            processed.append(video.name)
            continue

        failures.append(video.name)
        print(
            f'error: {video.name} failed with exit code '
            f'{result.returncode}',
            file=sys.stderr,
        )
        if result.returncode == 130:
            break
        if not args.continue_on_error:
            break

    print(
        f'\nBatch summary: {len(processed)} processed, '
        f'{len(skipped)} skipped as valid, {len(failures)} failed.'
    )
    if failures:
        print(f"Failed videos: {', '.join(failures)}", file=sys.stderr)
        return 1
    print(f'All outputs are validated in: {output_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

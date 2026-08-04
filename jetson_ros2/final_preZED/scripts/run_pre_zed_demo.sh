#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
CONFIG_FILE="${REPO_ROOT}/final_preZED/configs/pre_zed_params.yaml"
WORKSPACE_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"

if grep -Eq '^[[:space:]]*detector_model_id:[[:space:]]*(""|null)?[[:space:]]*$' \
  "${CONFIG_FILE}"; then
  echo "Configuration error: detector_model_id is empty in ${CONFIG_FILE}." >&2
  echo "Set it to the real Roboflow project/version model ID before launching." >&2
  exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble setup was not found at /opt/ros/humble/setup.bash." >&2
  exit 3
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "Workspace is not built. Run the documented colcon build first." >&2
  exit 4
fi

cd "${REPO_ROOT}"

# ROS 2 Humble setup scripts read optional variables that may be unset.
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_SETUP}"
set -u

exec ros2 launch pre_zed_perception_py pre_zed_pipeline.launch.py \
  config_file:="${CONFIG_FILE}"

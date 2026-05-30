#!/usr/bin/env bash

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

print_header() {
  echo
  echo "Docker Helper"
  echo "Repo root: ${REPO_ROOT}"
  echo "Tip: use q or exit to leave the menu."
  echo
}

is_quit() {
  [[ "${1:-}" == "q" || "${1:-}" == "quit" || "${1:-}" == "exit" ]]
}

list_images() {
  echo
  echo "Available images:"
  if ! docker image ls --format 'table {{.Repository}}\t{{.Tag}}\t{{.ID}}\t{{.CreatedSince}}'; then
    echo "Failed to list Docker images."
  fi
}

list_containers() {
  echo
  echo "Available containers:"
  if ! docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.RunningFor}}'; then
    echo "Failed to list Docker containers."
  fi
}

list_mounts() {
  echo
  echo "Container mount directories:"
  local names
  names="$(docker ps -a --format '{{.Names}}')"

  if [ -z "${names}" ]; then
    echo "No containers found."
    return
  fi

  while IFS= read -r name; do
    [ -z "${name}" ] && continue
    echo
    echo "Container: ${name}"
    if ! docker inspect --format '{{if .Mounts}}{{range .Mounts}}- host: {{.Source}} -> container: {{.Destination}}{{println}}{{end}}{{else}}- no mounts configured{{end}}' "${name}"; then
      echo "- failed to inspect mounts"
    fi
  done <<< "${names}"
}

build_image() {
  local image_name dockerfile_path build_context

  echo
  read -r -p "Image name: " image_name
  if is_quit "${image_name}"; then
    return
  fi
  if [ -z "${image_name}" ]; then
    echo "Image name is required."
    return
  fi

  read -r -p "Dockerfile path [docker_Test/Dockerfile]: " dockerfile_path
  if is_quit "${dockerfile_path}"; then
    return
  fi

  read -r -p "Build context [.]: " build_context
  if is_quit "${build_context}"; then
    return
  fi

  dockerfile_path="${dockerfile_path:-docker_Test/Dockerfile}"
  build_context="${build_context:-.}"

  echo
  echo "Running build from repo root..."
  echo "docker build -f ${dockerfile_path} -t ${image_name} ${build_context}"

  (
    cd "${REPO_ROOT}" && \
    docker build -f "${dockerfile_path}" -t "${image_name}" "${build_context}"
  )
}

run_container() {
  local container_name image_name extra_args remove_response mount_response continue_response

  list_images
  list_containers

  echo
  echo "Recommended for the alive test:"
  echo "- image name: alive-ros2-test"
  echo "- container name: alive-ros2-container"
  echo
  read -r -p "Container name: " container_name
  if is_quit "${container_name}"; then
    return
  fi
  if [ -z "${container_name}" ]; then
    echo "Container name is required."
    return
  fi

  read -r -p "Image name to run: " image_name
  if is_quit "${image_name}"; then
    return
  fi
  if [ -z "${image_name}" ]; then
    echo "Image name is required."
    return
  fi

  if [ "${container_name}" = "${image_name}" ]; then
    echo
    echo "Warning: the container name is the same as the image name."
    echo "That works, but it can become confusing later."
    read -r -p "Continue anyway? [y/N]: " continue_response
    if [[ ! "${continue_response}" =~ ^[Yy]$ ]]; then
      echo "Run cancelled. Try again with different names."
      return
    fi
  fi

  read -r -p "Extra docker run args [none]: " extra_args
  if is_quit "${extra_args}"; then
    return
  fi

  echo
  echo "Default mode is interactive (-it)."
  read -r -p "Add --rm for auto removal? [y/N]: " remove_response
  if is_quit "${remove_response}"; then
    return
  fi

  echo
  echo "Mount mode is useful during development."
  echo "It mounts host ros2_ws/src into /workspace/ros2_ws/src inside the container."
  read -r -p "Mount host ros2_ws/src for live source edits? [y/N]: " mount_response
  if is_quit "${mount_response}"; then
    return
  fi

  local cmd=(docker run -it --name "${container_name}")
  if [[ "${remove_response}" =~ ^[Yy]$ ]]; then
    cmd+=(--rm)
  fi

  if [[ "${mount_response}" =~ ^[Yy]$ ]]; then
    cmd+=(-v "${REPO_ROOT}/ros2_ws/src:/workspace/ros2_ws/src")
  fi

  if [ -n "${extra_args}" ]; then
    # shellcheck disable=SC2206
    local extra_parts=( ${extra_args} )
    cmd+=("${extra_parts[@]}")
  fi

  cmd+=("${image_name}")

  echo
  printf 'Running:'
  printf ' %q' "${cmd[@]}"
  printf '\n'

  "${cmd[@]}"
}

exec_container() {
  local container_name shell_name

  list_containers

  echo
  read -r -p "Container name to enter: " container_name
  if is_quit "${container_name}"; then
    return
  fi
  if [ -z "${container_name}" ]; then
    echo "Container name is required."
    return
  fi

  read -r -p "Shell to open [bash]: " shell_name
  if is_quit "${shell_name}"; then
    return
  fi

  shell_name="${shell_name:-bash}"

  echo
  echo "Running: docker exec -it ${container_name} ${shell_name}"
  docker exec -it "${container_name}" "${shell_name}"
}

show_all() {
  list_images
  list_containers
  list_mounts
}

main_menu() {
  local choice

  while true; do
    print_header
    echo "Choose an action:"
    echo "1. Build image"
    echo "2. Run container"
    echo "3. Exec into running container"
    echo "4. List images, containers, and mount directories"
    echo "5. Exit"
    echo
    read -r -p "Selection [1-5, q]: " choice

    case "${choice}" in
      1) build_image ;;
      2) run_container ;;
      3) exec_container ;;
      4) show_all ;;
      5|q|quit|exit) exit 0 ;;
      *) echo "Invalid option. Choose 1, 2, 3, 4, 5, or q." ;;
    esac
  done
}

main_menu

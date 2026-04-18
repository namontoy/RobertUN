#!/usr/bin/env bash

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

print_header() {
  echo
  echo "Docker Helper"
  echo "Repo root: ${REPO_ROOT}"
  echo
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
  read -r -p "Dockerfile path [docker_Test/Dockerfile]: " dockerfile_path
  read -r -p "Build context [.]: " build_context

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
  local container_name image_name extra_args response

  list_images

  echo
  read -r -p "Container name: " container_name
  read -r -p "Image name to run: " image_name
  read -r -p "Extra docker run args [none]: " extra_args

  echo
  echo "Default mode is interactive (-it)."
  read -r -p "Add --rm for auto removal? [y/N]: " response

  local cmd=(docker run -it --name "${container_name}")
  if [[ "${response}" =~ ^[Yy]$ ]]; then
    cmd+=(--rm)
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
  read -r -p "Shell to open [bash]: " shell_name

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
    read -r -p "Selection [1-5]: " choice

    case "${choice}" in
      1) build_image ;;
      2) run_container ;;
      3) exec_container ;;
      4) show_all ;;
      5) exit 0 ;;
      *) echo "Invalid option. Choose 1, 2, 3, 4, or 5." ;;
    esac
  done
}

main_menu

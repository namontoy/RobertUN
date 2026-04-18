# minimal test docker and ros2

This note explains the first Docker test for this repo.

The goal is simple:

- build the `alive_py` ROS 2 package inside Docker
- run one container
- open two shells in that same container
- in one shell run the node
- in the other shell echo the topic
- confirm the node prints `message from the docker`



## Files used

- `docker_Test/Dockerfile`
- `docker_Test/.dockerignore`
- `ros2_ws/src/alive_py/alive_py/alive_node.py`
- `scripts/docker_scripts/docker_menu.sh`


## What the Dockerfile does

File:

`docker_Test/Dockerfile`

Full file:

<pre>
FROM ros:humble-ros-base

# Use bash so ROS setup scripts can be sourced in RUN commands.
SHELL ["/bin/bash", "-c"]

# Install colcon so the workspace can be built inside the container.
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# Create the workspace path that will exist inside the image.
WORKDIR /workspace

# Copy only the ROS 2 source package into the image for this minimal test.
COPY ros2_ws/src ./ros2_ws/src

# Build the workspace so alive_py is available at runtime.
RUN source /opt/ros/humble/setup.bash && \
    cd /workspace/ros2_ws && \
    colcon build --symlink-install

# Default to an interactive shell with both ROS environments sourced.
CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && exec bash"]
</pre>

### Explaining the Dockerile

`FROM ros:humble-ros-base`

- Starts from an official ROS 2 Humble base image.
- This gives the container a ready ROS 2 installation.

`SHELL ["/bin/bash", "-c"]`

- Tells Docker to use `bash` for the build commands.
- We do this because ROS setup scripts are commonly sourced from `bash` during image build steps.

`RUN apt-get update && apt-get install -y python3-colcon-common-extensions`

- Installs `colcon`, the ROS 2 build tool.
- Without this, the container would have ROS 2 but not the workspace build tool we need.

`rm -rf /var/lib/apt/lists/*`

- Cleans apt metadata after installation.
- This keeps the image smaller.

`WORKDIR /workspace`

- Sets the default working directory inside the image.
- All following relative paths are based from here.

`COPY ros2_ws/src ./ros2_ws/src`

- Copies only the source packages into the image.
- We do not copy `build/`, `install/`, or `log/` because those are generated artifacts.

`RUN source /opt/ros/humble/setup.bash && cd /workspace/ros2_ws && colcon build --symlink-install`

- Loads the base ROS 2 environment.
- Moves into the workspace.
- Builds the workspace so `alive_py` becomes available in the image.

`CMD ["/bin/bash", "-lc", "..."]`

- Defines the default command when the container starts.
- It opens a shell where both environments are already sourced:
  - base ROS 2
  - the built workspace
- `exec bash` keeps the shell open for interactive use.

---

## What `.dockerignore` does

File:

`docker_Test/.dockerignore`

This file tells Docker which host files should not be sent into the Docker build context.

Why that matters:

- smaller build context
- faster image builds
- less accidental copying of generated files

Important ignored items here:

- `.git`
- notes and docs not needed for the image
- `ros2_ws/build`
- `ros2_ws/install`
- `ros2_ws/log`
- Python cache files

The image should be built from source, not from old generated workspace artifacts.

---

## Workflow

### 1. Build the image

Run this from the repo root:

<pre>
cd ~/RobertUN/jetson_ros2
docker build -f docker_Test/Dockerfile -t alive-ros2-test .
</pre>

What this does:

- uses `docker_Test/Dockerfile`
- builds an image named `alive-ros2-test`
- uses the repo root as the build context

Why the repo root is the build context:

- the Dockerfile copies `ros2_ws/src`
- that path exists relative to the repo root

### 2. Start one interactive container

<pre>
docker run -it --name alive-ros2-container alive-ros2-test
</pre>

What this does:

- starts one container from the image
- gives you an interactive shell
- because of the `CMD`, the shell already has ROS 2 and the workspace sourced

### 3. Open a second shell into the same running container

From another terminal on the host:

<pre>
docker exec -it alive-ros2-container bash
</pre>

This opens another shell in the same container.

For this second shell, source the environments again:

<pre>
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
</pre>

This is needed because each shell has its own environment.

### 4. Run the node in the first shell

Inside the container shell that started with `docker run`:

<pre>
ros2 run alive_py alive_node
</pre>

Expected behavior:

- the node starts
- it publishes on `/alive`
- it logs the startup line
- it prints `message from the docker`

### 5. Echo the topic in the second shell

Inside the second container shell:

<pre>
ros2 topic echo /alive
</pre>

Expected behavior:

<pre>
data: true
---
data: true
---
</pre>

That means the publisher and subscriber tools are communicating inside the same container.

## Docker helper

We also created an interactive helper script:

`scripts/docker_scripts/docker_menu.sh`

This helper is meant to reduce typing for future Docker work in this repo.

What it can do:

- build an image
- run a container
- exec into a running container
- list current images, containers, and container mount directories

This helper is explained in detail in:

`Notes/helper_docker.md`


## How to understand this execution

The execution path is:

1. Docker starts a container from the built image.
2. The image already contains the built ROS 2 workspace.
3. `ros2 run alive_py alive_node` starts the Python executable registered in `setup.py`.
4. The node publishes `Bool(data=True)` on `/alive`.
5. `ros2 topic echo /alive` subscribes to that topic and prints each message.

This test proves:

- Docker can hold a ROS 2 environment
- the workspace can be built inside Docker
- your package can run inside Docker
- ROS 2 topic communication works in that container



## Next logical step after this

After this minimal test works, the next useful Docker step would be one of these:

- create two containers for the echo and the alive
- mount the workspace from the host instead of copying it into the image
- expand the helper to support stop, remove, and mount-based development workflows
- test ROS 2 GUI tools or networking

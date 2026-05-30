FROM ros:humble-ros-base

# Use bash so ROS setup scripts can be sourced in RUN commands.
SHELL ["/bin/bash", "-c"]

# Install colcon and OpenCV for the image-first perception nodes.
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-opencv \
 && rm -rf /var/lib/apt/lists/*

# Create the workspace path that will exist inside the image.
WORKDIR /workspace

# Copy only the ROS 2 source package into the image for this minimal test.
COPY ros2_ws/src ./ros2_ws/src

# Build the workspace so the ROS 2 Python packages are available at runtime.
RUN source /opt/ros/humble/setup.bash && \
    cd /workspace/ros2_ws && \
    colcon build --symlink-install

# Default to an interactive shell with both ROS environments sourced.
CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && exec bash"]

# Image-first color car counter

This note explains the first perception step without the ZED camera.

The goal is to use a local traffic image and count likely cars by color through a ROS 2 node.

This is not real object detection yet.
It is a color-segmentation exercise that helps us practice ROS 2 perception flow before the camera arrives.

## Package

Package:

`ros2_ws/src/image_tools_py`

Service interface package:

`ros2_ws/src/image_tools_interfaces`

Node:

`color_car_counter`

Service node:

`color_car_counter_service`

Source file:

`ros2_ws/src/image_tools_py/image_tools_py/color_car_counter_node.py`

Service source file:

`ros2_ws/src/image_tools_py/image_tools_py/color_car_counter_service_node.py`

## What the node does

The node:

1. Loads one image from disk.
2. Converts the image from BGR to HSV color space.
3. Builds a mask for one selected color.
4. Cleans the mask with OpenCV morphology.
5. Counts contour regions that look large enough to be car-colored regions.
6. Logs the count.
7. Publishes the count on `/color_car_count`.
8. Publishes a text summary on `/color_car_summary`.

## Supported colors

The parameter `target_color` accepts:

- `red`
- `white`
- `black`

## Topics

`/color_car_count`

- Type: `std_msgs/msg/Int32`
- Meaning: estimated number of likely cars matching the selected color

`/color_car_summary`

- Type: `std_msgs/msg/String`
- Meaning: readable summary of the result

## Parameters

`image_path`

- Required.
- Path to the local image file.

`target_color`

- Default: `red`
- Options: `red`, `white`, `black`

`min_area`

- Default: `450.0`
- Smaller values count smaller regions.
- Larger values ignore more small fragments.

`publish_rate_hz`

- Default: `1.0`
- Controls how often the result is published.

## Service workflow

This is the more useful interactive workflow.

Terminal 1 runs a service server and waits.
It does not process the image until another terminal sends a request.

Service:

`/count_cars_by_color`

Service type:

`image_tools_interfaces/srv/CountCarsByColor`

Request fields:

- `image_path`
- `target_color`
- `save_output_image`
- `output_dir`

Response fields:

- `success`
- `count`
- `summary`
- `output_image_path`

### Terminal 1: start the service

<pre>
cd ~/RobertUN/jetson_ros2/ros2_ws
colcon build --symlink-install
ros2ws_init
ros2 run image_tools_py color_car_counter_service
</pre>

Expected startup log:

<pre>
Ready: /count_cars_by_color
</pre>

### Terminal 2: use the helper menu

<pre>
cd ~/RobertUN/jetson_ros2
ros2ws_init
bash scripts/image_tools/request_color_count.sh
</pre>

The helper asks:

<pre>
Image path [/home/ingfisica/RobertUN/jetson_ros2/images/cars_Test.png]:
Choose target color:
1. red
2. white
3. black
Save annotated output image? [Y/n]:
Output directory [same as input image]:
</pre>

If output saving is enabled and the output directory is empty, the annotated image is saved next to the input image.

Example output path:

<pre>
/home/ingfisica/RobertUN/jetson_ros2/images/cars_Test_red_detected.png
</pre>

### Direct service call

The helper is only a convenience.
The direct ROS 2 command is:

<pre>
ros2 service call /count_cars_by_color image_tools_interfaces/srv/CountCarsByColor \
  "{image_path: '/home/ingfisica/RobertUN/jetson_ros2/images/cars_Test.png', target_color: 'red', save_output_image: true, output_dir: ''}"
</pre>

## How to run

This older workflow processes immediately at node startup.
Use it when you want one quick run without the service/helper flow.

Save the traffic image somewhere on the machine first.

Recommended local path:

<pre>
~/RobertUN/jetson_ros2/images/cars_Test.png
</pre>

The `images/` folder is where the current traffic test image is stored.

Build the workspace:

<pre>
cd ~/RobertUN/jetson_ros2/ros2_ws
colcon build --symlink-install
ros2ws_init
</pre>

Run the detector:

<pre>
ros2 run image_tools_py color_car_counter --ros-args \
  -p image_path:=~/RobertUN/jetson_ros2/images/cars_Test.png \
  -p target_color:=red
</pre>

In a second terminal:

<pre>
ros2ws_init
ros2 topic echo /color_car_count
</pre>

To change color:

<pre>
ros2 run image_tools_py color_car_counter --ros-args \
  -p image_path:=~/RobertUN/jetson_ros2/images/cars_Test.png \
  -p target_color:=white
</pre>

Or:

<pre>
ros2 run image_tools_py color_car_counter --ros-args \
  -p image_path:=~/RobertUN/jetson_ros2/images/cars_Test.png \
  -p target_color:=black
</pre>

## Docker development

Use the Docker helper with mount mode enabled if you want the container to see host source edits:

<pre>
bash scripts/docker_scripts/docker_menu.sh
</pre>

Choose run container and answer:

<pre>
Mount host ros2_ws/src for live source edits? [y/N]: y
</pre>

Then rebuild the workspace inside the container because this is a new package:

<pre>
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
</pre>

To use the local image inside Docker, also mount the image folder when running the container.

In the helper prompt:

<pre>
Extra docker run args [none]: -v /home/ingfisica/RobertUN/jetson_ros2/images:/workspace/images
</pre>

Then inside Docker use:

<pre>
ros2 run image_tools_py color_car_counter --ros-args \
  -p image_path:=/workspace/images/cars_Test.png \
  -p target_color:=red
</pre>

## Current limitation

This node detects color regions, not true cars.

For the provided traffic image, red/white/black car counts are estimates.
The next stage can replace this with a real detector while keeping the ROS 2 parameter and topic workflow.

## Red detection tuning

After inspecting the annotated red output image, the detector was tightened to remove obvious false positives.

The improved filter now rejects:

- very small boxes
- boxes cut off by the bottom edge of the image
- shapes with weak filled area inside the bounding box
- shapes with aspect ratios that are less car-like

The annotated image also includes a label with the selected color and count.

This is still a heuristic.
It improves the red demo, but it does not make white or black reliable because those colors overlap heavily with road markings, reflections, shadows, windows, and tires.

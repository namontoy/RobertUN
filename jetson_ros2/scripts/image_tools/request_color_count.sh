#!/usr/bin/env bash

set -u

DEFAULT_IMAGE="/home/ingfisica/RobertUN/jetson_ros2/images/cars_Test.png"
SERVICE_NAME="/count_cars_by_color"
SERVICE_TYPE="image_tools_interfaces/srv/CountCarsByColor"

echo
echo "Color Car Count Request"
echo
read -r -p "Image path [${DEFAULT_IMAGE}]: " image_path
image_path="${image_path:-${DEFAULT_IMAGE}}"

echo
echo "Choose target color:"
echo "1. red"
echo "2. white"
echo "3. black"
read -r -p "Selection [1-3]: " color_choice

case "${color_choice}" in
  1) target_color="red" ;;
  2) target_color="white" ;;
  3) target_color="black" ;;
  red|white|black) target_color="${color_choice}" ;;
  *)
    echo "Invalid color selection. Use 1, 2, 3, red, white, or black."
    exit 1
    ;;
esac

read -r -p "Save annotated output image? [Y/n]: " save_choice
save_output_image=true
if [[ "${save_choice}" =~ ^[Nn]$ ]]; then
  save_output_image=false
fi

read -r -p "Output directory [same as input image]: " output_dir

echo
echo "Calling ${SERVICE_NAME}..."
request="{image_path: '${image_path}', target_color: '${target_color}', "
request+="save_output_image: ${save_output_image}, output_dir: '${output_dir}'}"

ros2 service call "${SERVICE_NAME}" "${SERVICE_TYPE}" \
  "${request}"

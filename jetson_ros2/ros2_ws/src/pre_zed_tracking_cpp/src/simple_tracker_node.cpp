// Copyright 2026 ingfisica
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "pre_zed_tracking_cpp/tracker.hpp"

namespace
{

using boost::property_tree::ptree;
using pre_zed_tracking_cpp::BoundingBox;
using pre_zed_tracking_cpp::Detection;
using pre_zed_tracking_cpp::MultiObjectTracker;
using pre_zed_tracking_cpp::Track;

struct DetectionFrame
{
  std::uint64_t stamp_ns;
  std::string frame_id;
  int image_width;
  int image_height;
  std::string model_id;
  double detector_inference_ms;
  std::vector<Detection> detections;
};

std::string escape_json(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        } else {
          output << character;
        }
    }
  }
  return output.str();
}

BoundingBox parse_bbox(const ptree & node)
{
  std::vector<double> values;
  for (const auto & item : node) {
    values.push_back(item.second.get_value<double>());
  }
  if (values.size() != 4) {
    throw std::runtime_error("bbox_xywh must contain exactly four numbers");
  }
  return {values[0], values[1], values[2], values[3]};
}

DetectionFrame parse_detection_frame(const std::string & json)
{
  std::istringstream input(json);
  ptree root;
  boost::property_tree::read_json(input, root);
  if (root.get<std::string>("schema_version") != "0.1.0") {
    throw std::runtime_error("unsupported detection schema_version");
  }

  DetectionFrame frame{
    root.get<std::uint64_t>("stamp_ns"),
    root.get<std::string>("frame_id", ""),
    root.get<int>("image_width"),
    root.get<int>("image_height"),
    root.get<std::string>("model_id", ""),
    root.get<double>("inference_ms", 0.0),
    {},
  };
  for (const auto & item : root.get_child("detections")) {
    const auto & node = item.second;
    frame.detections.push_back(
      {
        node.get<std::string>("label"),
        node.get<int>("class_id", -1),
        node.get<double>("confidence"),
        parse_bbox(node.get_child("bbox_xywh")),
      });
  }
  return frame;
}

std::string serialize_track_frame(
  const DetectionFrame & frame, const std::vector<Track> & tracks)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(6);
  output << "{\"schema_version\":\"0.1.0\""
         << ",\"stamp_ns\":" << frame.stamp_ns
         << ",\"frame_id\":\"" << escape_json(frame.frame_id) << "\""
         << ",\"image_width\":" << frame.image_width
         << ",\"image_height\":" << frame.image_height
         << ",\"detector_model_id\":\"" << escape_json(frame.model_id) << "\""
         << ",\"detector_inference_ms\":" << frame.detector_inference_ms
         << ",\"tracks\":[";
  for (std::size_t index = 0; index < tracks.size(); ++index) {
    const auto & track = tracks[index];
    if (index > 0) {
      output << ',';
    }
    output << "{\"track_id\":" << track.id
           << ",\"label\":\"" << escape_json(track.label) << "\""
           << ",\"class_id\":" << track.class_id
           << ",\"confidence\":" << track.confidence
           << ",\"bbox_xywh\":["
           << track.bbox.x << ',' << track.bbox.y << ','
           << track.bbox.width << ',' << track.bbox.height << ']'
           << ",\"age\":" << track.age
           << ",\"missed_frames\":" << track.missed_frames
           << ",\"observed\":" << (track.observed ? "true" : "false")
           << '}';
  }
  output << "]}";
  return output.str();
}

}  // namespace

class SimpleTrackerNode : public rclcpp::Node
{
public:
  SimpleTrackerNode()
  : Node("simple_tracker_node"),
    tracker_(
      declare_parameter<double>("iou_threshold", 0.3),
      declare_parameter<int>("max_missed_frames", 3))
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/prezed/detections");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/prezed/tracks");
    publisher_ = create_publisher<std_msgs::msg::String>(output_topic, 10);
    subscription_ = create_subscription<std_msgs::msg::String>(
      input_topic,
      10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        on_detections(*message);
      });
    RCLCPP_INFO(
      get_logger(), "Tracking %s and publishing %s.",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void on_detections(const std_msgs::msg::String & message)
  {
    try {
      const auto frame = parse_detection_frame(message.data);
      const auto tracks = tracker_.update(frame.detections);
      std_msgs::msg::String output;
      output.data = serialize_track_frame(frame, tracks);
      publisher_->publish(output);
      RCLCPP_INFO(
        get_logger(), "Frame %" PRIu64 ": %zu detections, %zu active tracks.",
        frame.stamp_ns,
        frame.detections.size(), tracks.size());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Invalid detection payload: %s", error.what());
    }
  }

  MultiObjectTracker tracker_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

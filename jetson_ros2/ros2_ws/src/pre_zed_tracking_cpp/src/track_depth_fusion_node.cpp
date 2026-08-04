// Copyright 2026 ingfisica
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <cinttypes>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

using boost::property_tree::ptree;

struct BoundingBox
{
  double x;
  double y;
  double width;
  double height;
};

struct Track
{
  std::uint64_t id;
  std::string label;
  int class_id;
  double confidence;
  BoundingBox bbox;
  int age;
  int missed_frames;
  bool observed;
};

struct TrackFrame
{
  std::uint64_t stamp_ns;
  std::string frame_id;
  int image_width;
  int image_height;
  std::string detector_model_id;
  double detector_inference_ms;
  std::vector<Track> tracks;
};

struct DepthFrame
{
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t step;
  std::vector<std::uint8_t> data;
};

struct DepthStatistic
{
  bool valid{false};
  double median{0.0};
  std::size_t sample_count{0};
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

TrackFrame parse_track_frame(const std::string & json)
{
  std::istringstream input(json);
  ptree root;
  boost::property_tree::read_json(input, root);
  if (root.get<std::string>("schema_version") != "0.1.0") {
    throw std::runtime_error("unsupported track schema_version");
  }
  TrackFrame frame{
    root.get<std::uint64_t>("stamp_ns"),
    root.get<std::string>("frame_id", ""),
    root.get<int>("image_width"),
    root.get<int>("image_height"),
    root.get<std::string>("detector_model_id", ""),
    root.get<double>("detector_inference_ms", 0.0),
    {},
  };
  for (const auto & item : root.get_child("tracks")) {
    const auto & node = item.second;
    frame.tracks.push_back(
      {
        node.get<std::uint64_t>("track_id"),
        node.get<std::string>("label"),
        node.get<int>("class_id", -1),
        node.get<double>("confidence"),
        parse_bbox(node.get_child("bbox_xywh")),
        node.get<int>("age", 1),
        node.get<int>("missed_frames", 0),
        node.get<bool>("observed", true),
      });
  }
  return frame;
}

std::uint64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.nanosec);
}

DepthStatistic median_in_bbox(
  const DepthFrame & depth, const BoundingBox & bbox, double inner_scale,
  std::size_t minimum_samples)
{
  const double scale = std::clamp(inner_scale, 0.1, 1.0);
  const double inset_x = bbox.width * (1.0 - scale) * 0.5;
  const double inset_y = bbox.height * (1.0 - scale) * 0.5;
  const int left = std::clamp(
    static_cast<int>(std::floor(bbox.x + inset_x)), 0,
    static_cast<int>(depth.width));
  const int top = std::clamp(
    static_cast<int>(std::floor(bbox.y + inset_y)), 0,
    static_cast<int>(depth.height));
  const int right = std::clamp(
    static_cast<int>(std::ceil(bbox.x + bbox.width - inset_x)), 0,
    static_cast<int>(depth.width));
  const int bottom = std::clamp(
    static_cast<int>(std::ceil(bbox.y + bbox.height - inset_y)), 0,
    static_cast<int>(depth.height));

  std::vector<std::uint8_t> values;
  if (right <= left || bottom <= top) {
    return {};
  }
  values.reserve(static_cast<std::size_t>((right - left) * (bottom - top)));
  for (int y = top; y < bottom; ++y) {
    for (int x = left; x < right; ++x) {
      const auto value =
        depth.data[static_cast<std::size_t>(y) * depth.step + x];
      if (value != 0) {
        values.push_back(value);
      }
    }
  }
  if (values.size() < minimum_samples) {
    return {false, 0.0, values.size()};
  }

  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  double median = static_cast<double>(*middle);
  if (values.size() % 2 == 0) {
    const auto lower = std::max_element(values.begin(), middle);
    median = (median + static_cast<double>(*lower)) * 0.5;
  }

  // Zero is reserved for invalid pixels; valid values map from 1 through 255.
  const double normalized = std::clamp((median - 1.0) / 254.0, 0.0, 1.0);
  return {true, normalized, values.size()};
}

std::string serialize_fused_frame(
  const TrackFrame & frame, const DepthFrame & depth, double inner_scale,
  std::size_t minimum_samples)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(6);
  output << "{\"schema_version\":\"0.1.0\""
         << ",\"contract_status\":\"temporary_pre_zed\""
         << ",\"stamp_ns\":" << frame.stamp_ns
         << ",\"frame_id\":\"" << escape_json(frame.frame_id) << "\""
         << ",\"image_width\":" << frame.image_width
         << ",\"image_height\":" << frame.image_height
         << ",\"detector_model_id\":\"" << escape_json(frame.detector_model_id) << "\""
         << ",\"detector_inference_ms\":" << frame.detector_inference_ms
         << ",\"depth_source\":\"depth_anything_v3_relative\""
         << ",\"depth_encoding\":\"per_frame_minmax_mono8\""
         << ",\"depth_is_metric\":false"
         << ",\"tracks\":[";
  for (std::size_t index = 0; index < frame.tracks.size(); ++index) {
    const auto & track = frame.tracks[index];
    const auto statistic = median_in_bbox(
      depth, track.bbox, inner_scale, minimum_samples);
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
           << ",\"relative_depth_valid\":" << (statistic.valid ? "true" : "false")
           << ",\"relative_depth_median\":";
    if (statistic.valid) {
      output << statistic.median;
    } else {
      output << "null";
    }
    output << ",\"relative_depth_samples\":" << statistic.sample_count
           << ",\"depth_source\":\"depth_anything_v3_relative\""
           << '}';
  }
  output << "]}";
  return output.str();
}

}  // namespace

class TrackDepthFusionNode : public rclcpp::Node
{
public:
  TrackDepthFusionNode()
  : Node("track_depth_fusion_node")
  {
    const auto tracks_topic =
      declare_parameter<std::string>("tracks_topic", "/prezed/tracks");
    const auto depth_topic =
      declare_parameter<std::string>(
      "depth_topic", "/prezed/depth/relative_image");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/tracked_objects");
    inner_bbox_scale_ = declare_parameter<double>("inner_bbox_scale", 0.6);
    minimum_depth_samples_ = static_cast<std::size_t>(
      std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("minimum_depth_samples", 20)));
    cache_size_ = static_cast<std::size_t>(
      std::max<std::int64_t>(
        2, declare_parameter<std::int64_t>("cache_size", 30)));

    publisher_ = create_publisher<std_msgs::msg::String>(output_topic, 10);
    tracks_subscription_ = create_subscription<std_msgs::msg::String>(
      tracks_topic,
      10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        on_tracks(*message);
      });

    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic,
      image_qos,
      [this](const sensor_msgs::msg::Image::SharedPtr message) {
        on_depth(*message);
      });
    RCLCPP_INFO(
      get_logger(), "Fusing %s and %s into %s.",
      tracks_topic.c_str(), depth_topic.c_str(), output_topic.c_str());
  }

private:
  void on_tracks(const std_msgs::msg::String & message)
  {
    try {
      auto frame = parse_track_frame(message.data);
      const auto stamp_ns = frame.stamp_ns;
      track_frames_[stamp_ns] = std::move(frame);
      track_order_.push_back(stamp_ns);
      trim_cache(track_frames_, track_order_);
      try_fuse(stamp_ns);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Invalid track payload: %s", error.what());
    }
  }

  void on_depth(const sensor_msgs::msg::Image & message)
  {
    if (message.encoding != "mono8") {
      RCLCPP_ERROR(
        get_logger(), "Expected mono8 relative depth, received %s.",
        message.encoding.c_str());
      return;
    }
    const std::size_t required_size =
      static_cast<std::size_t>(message.height) * message.step;
    if (message.step < message.width || message.data.size() < required_size) {
      RCLCPP_ERROR(get_logger(), "Relative-depth image has invalid dimensions.");
      return;
    }

    const auto stamp_ns = stamp_to_nanoseconds(message.header.stamp);
    depth_frames_[stamp_ns] = {
      message.width,
      message.height,
      message.step,
      message.data,
    };
    depth_order_.push_back(stamp_ns);
    trim_cache(depth_frames_, depth_order_);
    try_fuse(stamp_ns);
  }

  template<typename Value>
  void trim_cache(
    std::unordered_map<std::uint64_t, Value> & cache,
    std::deque<std::uint64_t> & order)
  {
    while (order.size() > cache_size_) {
      const auto oldest = order.front();
      order.pop_front();
      cache.erase(oldest);
    }
  }

  void try_fuse(std::uint64_t stamp_ns)
  {
    const auto tracks = track_frames_.find(stamp_ns);
    const auto depth = depth_frames_.find(stamp_ns);
    if (tracks == track_frames_.end() || depth == depth_frames_.end()) {
      return;
    }

    std_msgs::msg::String output;
    output.data = serialize_fused_frame(
      tracks->second,
      depth->second,
      inner_bbox_scale_,
      minimum_depth_samples_);
    publisher_->publish(output);
    RCLCPP_INFO(
      get_logger(), "Published %zu fused tracks for frame %" PRIu64 ".",
      tracks->second.tracks.size(), stamp_ns);
    track_frames_.erase(tracks);
    depth_frames_.erase(depth);
  }

  double inner_bbox_scale_{0.6};
  std::size_t minimum_depth_samples_{20};
  std::size_t cache_size_{30};
  std::unordered_map<std::uint64_t, TrackFrame> track_frames_;
  std::unordered_map<std::uint64_t, DepthFrame> depth_frames_;
  std::deque<std::uint64_t> track_order_;
  std::deque<std::uint64_t> depth_order_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracks_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackDepthFusionNode>());
  rclcpp::shutdown();
  return 0;
}

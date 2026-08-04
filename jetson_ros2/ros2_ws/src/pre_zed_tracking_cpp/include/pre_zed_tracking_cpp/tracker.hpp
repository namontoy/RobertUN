// Copyright 2026 ingfisica
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef PRE_ZED_TRACKING_CPP__TRACKER_HPP_
#define PRE_ZED_TRACKING_CPP__TRACKER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace pre_zed_tracking_cpp
{

struct BoundingBox
{
  double x{0.0};
  double y{0.0};
  double width{0.0};
  double height{0.0};
};

struct Detection
{
  std::string label;
  int class_id{-1};
  double confidence{0.0};
  BoundingBox bbox;
};

struct Track
{
  std::uint64_t id{0};
  std::string label;
  int class_id{-1};
  double confidence{0.0};
  BoundingBox bbox;
  int age{1};
  int missed_frames{0};
  bool observed{true};
};

double intersection_over_union(const BoundingBox & first, const BoundingBox & second);

class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(double iou_threshold = 0.3, int max_missed_frames = 3);

  std::vector<Track> update(const std::vector<Detection> & detections);

private:
  double iou_threshold_;
  int max_missed_frames_;
  std::uint64_t next_track_id_{1};
  std::vector<Track> tracks_;
};

}  // namespace pre_zed_tracking_cpp

#endif  // PRE_ZED_TRACKING_CPP__TRACKER_HPP_

// Copyright 2026 ingfisica
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "pre_zed_tracking_cpp/tracker.hpp"

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace pre_zed_tracking_cpp
{

double intersection_over_union(const BoundingBox & first, const BoundingBox & second)
{
  const double first_right = first.x + std::max(0.0, first.width);
  const double first_bottom = first.y + std::max(0.0, first.height);
  const double second_right = second.x + std::max(0.0, second.width);
  const double second_bottom = second.y + std::max(0.0, second.height);

  const double intersection_left = std::max(first.x, second.x);
  const double intersection_top = std::max(first.y, second.y);
  const double intersection_right = std::min(first_right, second_right);
  const double intersection_bottom = std::min(first_bottom, second_bottom);
  const double intersection_width = std::max(0.0, intersection_right - intersection_left);
  const double intersection_height = std::max(0.0, intersection_bottom - intersection_top);
  const double intersection_area = intersection_width * intersection_height;

  const double first_area = std::max(0.0, first.width) * std::max(0.0, first.height);
  const double second_area = std::max(0.0, second.width) * std::max(0.0, second.height);
  const double union_area = first_area + second_area - intersection_area;
  return union_area > 0.0 ? intersection_area / union_area : 0.0;
}

MultiObjectTracker::MultiObjectTracker(double iou_threshold, int max_missed_frames)
: iou_threshold_(iou_threshold), max_missed_frames_(max_missed_frames)
{
  if (iou_threshold_ < 0.0 || iou_threshold_ > 1.0) {
    throw std::invalid_argument("iou_threshold must be between zero and one");
  }
  if (max_missed_frames_ < 0) {
    throw std::invalid_argument("max_missed_frames must not be negative");
  }
}

std::vector<Track> MultiObjectTracker::update(const std::vector<Detection> & detections)
{
  struct Candidate
  {
    double iou;
    std::size_t track_index;
    std::size_t detection_index;
  };

  std::vector<Candidate> candidates;
  for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
    for (std::size_t detection_index = 0; detection_index < detections.size();
      ++detection_index)
    {
      if (tracks_[track_index].label != detections[detection_index].label) {
        continue;
      }
      const double iou = intersection_over_union(
        tracks_[track_index].bbox, detections[detection_index].bbox);
      if (iou >= iou_threshold_) {
        candidates.push_back({iou, track_index, detection_index});
      }
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & first, const Candidate & second) {
      return first.iou > second.iou;
    });

  std::vector<bool> matched_tracks(tracks_.size(), false);
  std::vector<bool> matched_detections(detections.size(), false);
  for (const auto & candidate : candidates) {
    if (matched_tracks[candidate.track_index] ||
      matched_detections[candidate.detection_index])
    {
      continue;
    }
    matched_tracks[candidate.track_index] = true;
    matched_detections[candidate.detection_index] = true;
    auto & track = tracks_[candidate.track_index];
    const auto & detection = detections[candidate.detection_index];
    track.label = detection.label;
    track.class_id = detection.class_id;
    track.confidence = detection.confidence;
    track.bbox = detection.bbox;
    track.age += 1;
    track.missed_frames = 0;
    track.observed = true;
  }

  for (std::size_t index = 0; index < tracks_.size(); ++index) {
    if (!matched_tracks[index]) {
      tracks_[index].age += 1;
      tracks_[index].missed_frames += 1;
      tracks_[index].observed = false;
    }
  }

  for (std::size_t index = 0; index < detections.size(); ++index) {
    if (matched_detections[index]) {
      continue;
    }
    const auto & detection = detections[index];
    tracks_.push_back(
      {
        next_track_id_++,
        detection.label,
        detection.class_id,
        detection.confidence,
        detection.bbox,
        1,
        0,
        true,
      });
  }

  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & track) {
        return track.missed_frames > max_missed_frames_;
      }),
    tracks_.end());
  return tracks_;
}

}  // namespace pre_zed_tracking_cpp

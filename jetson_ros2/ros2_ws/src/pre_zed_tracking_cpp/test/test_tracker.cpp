// Copyright 2026 ingfisica
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <vector>

#include "pre_zed_tracking_cpp/tracker.hpp"

using pre_zed_tracking_cpp::BoundingBox;
using pre_zed_tracking_cpp::Detection;
using pre_zed_tracking_cpp::MultiObjectTracker;
using pre_zed_tracking_cpp::intersection_over_union;

TEST(IntersectionOverUnion, HandlesOverlap)
{
  const BoundingBox first{0.0, 0.0, 10.0, 10.0};
  const BoundingBox second{5.0, 0.0, 10.0, 10.0};
  EXPECT_NEAR(intersection_over_union(first, second), 1.0 / 3.0, 1e-9);
}

TEST(MultiObjectTracker, PreservesIdForMatchingClassAndBox)
{
  MultiObjectTracker tracker(0.2, 2);
  const auto first = tracker.update(
  {
    Detection{"car", 2, 0.9, {0.0, 0.0, 20.0, 20.0}},
  });
  const auto second = tracker.update(
  {
    Detection{"car", 2, 0.8, {2.0, 0.0, 20.0, 20.0}},
  });
  ASSERT_EQ(first.size(), 1U);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(first[0].id, second[0].id);
  EXPECT_EQ(second[0].age, 2);
  EXPECT_TRUE(second[0].observed);
}

TEST(MultiObjectTracker, DoesNotMatchDifferentClasses)
{
  MultiObjectTracker tracker(0.2, 2);
  const auto first = tracker.update(
  {
    Detection{"car", 2, 0.9, {0.0, 0.0, 20.0, 20.0}},
  });
  const auto second = tracker.update(
  {
    Detection{"person", 0, 0.8, {0.0, 0.0, 20.0, 20.0}},
  });
  ASSERT_EQ(first.size(), 1U);
  ASSERT_EQ(second.size(), 2U);
  EXPECT_NE(second[0].id, second[1].id);
}

TEST(MultiObjectTracker, RemovesExpiredTracks)
{
  MultiObjectTracker tracker(0.2, 1);
  tracker.update(
  {
    Detection{"car", 2, 0.9, {0.0, 0.0, 20.0, 20.0}},
  });
  const auto missed_once = tracker.update({});
  const auto expired = tracker.update({});
  ASSERT_EQ(missed_once.size(), 1U);
  EXPECT_FALSE(missed_once[0].observed);
  EXPECT_TRUE(expired.empty());
}

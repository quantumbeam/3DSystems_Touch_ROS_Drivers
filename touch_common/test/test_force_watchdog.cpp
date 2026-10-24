#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>

#include "touch_common/force_watchdog.hpp"

using touch_common::Force3;
using touch_common::ForceFeedbackBuffer;
using touch_common::force_within_deadline;
using touch_common::limit_force_norm;
using namespace std::chrono_literals;

constexpr int64_t kTimeoutNs = 100'000'000;  // 100 ms

TEST(LimitForceNorm, KeepsSmallForce)
{
  const Force3 out = limit_force_norm({1.0, 0.0, 0.0}, 3.3);
  EXPECT_DOUBLE_EQ(out[0], 1.0);
}

TEST(LimitForceNorm, ScalesLargeForceKeepingDirection)
{
  const Force3 out = limit_force_norm({3.0, 4.0, 0.0}, 2.5);  // |f| = 5 N
  EXPECT_NEAR(std::hypot(out[0], out[1]), 2.5, 1e-12);
  EXPECT_NEAR(out[0] / out[1], 0.75, 1e-12);
}

TEST(LimitForceNorm, NonFiniteBecomesZero)
{
  const double inf = std::numeric_limits<double>::infinity();
  const Force3 out = limit_force_norm({inf, 0.0, 0.0}, 3.3);
  EXPECT_EQ(out, (Force3{0.0, 0.0, 0.0}));
}

TEST(ForceWithinDeadline, NeverReceivedIsZero)
{
  EXPECT_EQ(force_within_deadline({1.0, 2.0, 3.0}, false, 0, kTimeoutNs), (Force3{0.0, 0.0, 0.0}));
}

TEST(ForceWithinDeadline, ExpiredIsZero)
{
  EXPECT_EQ(force_within_deadline({1.0, 2.0, 3.0}, true, kTimeoutNs + 1, kTimeoutNs),
            (Force3{0.0, 0.0, 0.0}));
  EXPECT_EQ(force_within_deadline({1.0, 2.0, 3.0}, true, kTimeoutNs, kTimeoutNs),
            (Force3{1.0, 2.0, 3.0}));
}

TEST(ForceWithinDeadline, NegativeAgeIsZero)
{
  EXPECT_EQ(force_within_deadline({1.0, 2.0, 3.0}, true, -1, kTimeoutNs), (Force3{0.0, 0.0, 0.0}));
}

TEST(ForceFeedbackBuffer, ZeroBeforeFirstMessage)
{
  ForceFeedbackBuffer buffer;
  EXPECT_EQ(buffer.load(kTimeoutNs), (Force3{0.0, 0.0, 0.0}));
  EXPECT_FALSE(buffer.fresh(kTimeoutNs));
}

TEST(ForceFeedbackBuffer, FreshForceIsAppliedThenExpires)
{
  ForceFeedbackBuffer buffer;
  const auto t0 = ForceFeedbackBuffer::Clock::now();
  buffer.store({0.5, -0.2, 1.0}, t0);
  EXPECT_EQ(buffer.load(kTimeoutNs, t0 + 50ms), (Force3{0.5, -0.2, 1.0}));
  EXPECT_TRUE(buffer.fresh(kTimeoutNs, t0 + 50ms));
  // 送り側が止まった: 期限を過ぎたら最後の力を出し続けない
  EXPECT_EQ(buffer.load(kTimeoutNs, t0 + 101ms), (Force3{0.0, 0.0, 0.0}));
  EXPECT_FALSE(buffer.fresh(kTimeoutNs, t0 + 101ms));
  // 新しい力が届けば再び出す
  buffer.store({0.1, 0.0, 0.0}, t0 + 200ms);
  EXPECT_EQ(buffer.load(kTimeoutNs, t0 + 210ms), (Force3{0.1, 0.0, 0.0}));
}

TEST(ForceFeedbackBuffer, NonFiniteComponentsAreZeroed)
{
  ForceFeedbackBuffer buffer;
  const auto t0 = ForceFeedbackBuffer::Clock::now();
  buffer.store({std::nan(""), std::numeric_limits<double>::infinity(), 0.3}, t0);
  EXPECT_EQ(buffer.load(kTimeoutNs, t0), (Force3{0.0, 0.0, 0.3}));
}

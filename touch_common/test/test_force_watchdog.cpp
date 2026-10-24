#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "touch_common/force_watchdog.hpp"

using touch_common::commanded_force;
using touch_common::Force3;
using touch_common::ForceFeedbackBuffer;
using touch_common::limit_force_norm;
using touch_common::valid_force_timeout_ms;
using touch_common::valid_max_force_n;
using namespace std::chrono_literals;

constexpr int64_t kTimeoutNs = 100'000'000;  // 100 ms
const Force3 kZero = {0.0, 0.0, 0.0};

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
  EXPECT_EQ(limit_force_norm({inf, 0.0, 0.0}, 3.3), kZero);
}

TEST(CommandedForce, AddsDampingWhenFresh)
{
  const Force3 out = commanded_force({1.0, 0.0, 0.0}, true, {100.0, 0.0, 0.0}, 0.001, 3.3);
  EXPECT_NEAR(out[0], 0.9, 1e-12);
}

TEST(CommandedForce, ExpiredIsZeroEvenWhileTheStylusMoves)
{
  // 送り側が止まったあとに人がスタイラスを動かしても、damping の力も出さない
  EXPECT_EQ(commanded_force({1.0, 0.0, 0.0}, false, {500.0, -300.0, 200.0}, 0.001, 3.3), kZero);
}

TEST(CommandedForce, TotalIsLimited)
{
  const Force3 out = commanded_force({3.0, 0.0, 0.0}, true, {-2000.0, 0.0, 0.0}, 0.001, 3.3);
  EXPECT_NEAR(out[0], 3.3, 1e-12);  // 3 N + 2 N の damping を 3.3 N に
}

TEST(ValidForceTimeout, RejectsOutOfRange)
{
  EXPECT_TRUE(valid_force_timeout_ms(100.0));
  EXPECT_TRUE(valid_force_timeout_ms(10000.0));
  EXPECT_FALSE(valid_force_timeout_ms(0.0));
  EXPECT_FALSE(valid_force_timeout_ms(-1.0));
  EXPECT_FALSE(valid_force_timeout_ms(1e20));  // ns に直すと int64_t に入らない
  EXPECT_FALSE(valid_force_timeout_ms(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(valid_force_timeout_ms(std::nan("")));
}

TEST(ForceFeedbackBuffer, ZeroBeforeFirstMessage)
{
  ForceFeedbackBuffer buffer;
  Force3 force = {9.0, 9.0, 9.0};
  EXPECT_FALSE(buffer.load(kTimeoutNs, force));
  EXPECT_EQ(force, kZero);
  EXPECT_FALSE(buffer.fresh(kTimeoutNs));
}

TEST(ForceFeedbackBuffer, FreshForceIsAppliedThenExpires)
{
  ForceFeedbackBuffer buffer;
  const auto t0 = ForceFeedbackBuffer::Clock::now();
  Force3 force;
  buffer.store({0.5, -0.2, 1.0}, t0);
  EXPECT_TRUE(buffer.load(kTimeoutNs, force, t0 + 50ms));
  EXPECT_EQ(force, (Force3{0.5, -0.2, 1.0}));
  EXPECT_TRUE(buffer.load(kTimeoutNs, force, t0 + 100ms));
  // 送り側が止まった: 期限を過ぎたら最後の力を出し続けない
  EXPECT_FALSE(buffer.load(kTimeoutNs, force, t0 + 101ms));
  EXPECT_EQ(force, kZero);
  EXPECT_FALSE(buffer.fresh(kTimeoutNs, t0 + 101ms));
  // 新しい力が届けば再び出す
  buffer.store({0.1, 0.0, 0.0}, t0 + 200ms);
  EXPECT_TRUE(buffer.load(kTimeoutNs, force, t0 + 210ms));
  EXPECT_EQ(force, (Force3{0.1, 0.0, 0.0}));
}

TEST(ForceFeedbackBuffer, ReceiptInTheFutureIsNotFresh)
{
  ForceFeedbackBuffer buffer;
  const auto t0 = ForceFeedbackBuffer::Clock::now();
  Force3 force;
  buffer.store({0.5, 0.0, 0.0}, t0 + 1s);
  EXPECT_FALSE(buffer.load(kTimeoutNs, force, t0));
}

TEST(ForceFeedbackBuffer, NonFiniteComponentsAreZeroed)
{
  ForceFeedbackBuffer buffer;
  const auto t0 = ForceFeedbackBuffer::Clock::now();
  Force3 force;
  buffer.store({std::nan(""), std::numeric_limits<double>::infinity(), 0.3}, t0);
  EXPECT_TRUE(buffer.load(kTimeoutNs, force, t0));
  EXPECT_EQ(force, (Force3{0.0, 0.0, 0.3}));
}

TEST(ValidMaxForce, RejectsAboveTheDeviceMaximum)
{
  EXPECT_TRUE(valid_max_force_n(3.3));
  EXPECT_TRUE(valid_max_force_n(1.0));
  EXPECT_FALSE(valid_max_force_n(3.31));
  EXPECT_FALSE(valid_max_force_n(100.0));
  EXPECT_FALSE(valid_max_force_n(0.0));
  EXPECT_FALSE(valid_max_force_n(std::nan("")));
}

TEST(ForceFeedbackBuffer, NeverMixesComponentsOfDifferentMessages)
{
  // 書き手が {3,0,0} と {0,3,0} を交互に書く間、読み手はどちらか（または 0）しか見ない
  ForceFeedbackBuffer buffer;
  std::atomic<bool> stop{false};
  std::thread writer([&] {
    for (int i = 0; !stop.load(); ++i) {
      buffer.store(i % 2 ? Force3{3.0, 0.0, 0.0} : Force3{0.0, 3.0, 0.0});
    }
  });
  int mixed = 0;
  int read = 0;
  const auto end = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < end) {
    Force3 force;
    if (buffer.load(kTimeoutNs, force)) {
      ++read;
      const bool x = force == Force3{3.0, 0.0, 0.0};
      const bool y = force == Force3{0.0, 3.0, 0.0};
      if (!x && !y) {
        ++mixed;
      }
    }
  }
  stop = true;
  writer.join();
  EXPECT_GT(read, 0);
  EXPECT_EQ(mixed, 0);
}

TEST(ShutdownZeroFrames, CountsOnlyCompletedZeroFramesAfterDisabling)
{
  using touch_common::counts_as_zero_frame;
  EXPECT_TRUE(counts_as_zero_frame(false, false, true));
  EXPECT_FALSE(counts_as_zero_frame(false, false, false));  // フレームがエラーで終わった
  EXPECT_FALSE(counts_as_zero_frame(true, false, true));    // 無効化前に計算した力を出したフレーム
  EXPECT_FALSE(counts_as_zero_frame(false, true, true));    // まだ出力中（受信が途切れていただけ）
}

TEST(ShutdownZeroFrames, ConfirmedWhenTheHapticLoopRuns)
{
  std::atomic<uint64_t> frames{5};
  std::atomic<bool> stop{false};
  std::thread haptic([&] {
    while (!stop.load()) {
      frames.fetch_add(1);
      std::this_thread::sleep_for(1ms);
    }
  });
  EXPECT_TRUE(touch_common::wait_for_zero_frames(frames, 5, 3, 50ms));
  stop = true;
  haptic.join();
}

TEST(ShutdownZeroFrames, GivesUpWhenTheHapticLoopHasStopped)
{
  std::atomic<uint64_t> frames{5};
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(touch_common::wait_for_zero_frames(frames, 5, 3, 50ms));
  const auto waited = std::chrono::steady_clock::now() - start;
  EXPECT_GE(waited, 50ms);
  EXPECT_LT(waited, 200ms);
}

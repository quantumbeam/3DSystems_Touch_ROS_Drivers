// Touch に出す力（force_feedback で受け取った力）を、受信の期限と大きさの上限で制限する。
//
// 以前のドライバは最後に受け取った力を出し続けたので、送り側（遠隔操作のノード）が落ちたり通信が
// 切れたりすると、Touch に力が残った。ここでは force_timeout_ms 以上新しい力が届かなければ 0 にする。
// OpenHaptics に依存しない（SDK のない環境でも単体テストできるように、touch_state.cpp から分けている）。

#ifndef TOUCH_COMMON__FORCE_WATCHDOG_HPP_
#define TOUCH_COMMON__FORCE_WATCHDOG_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace touch_common
{

using Force3 = std::array<double, 3>;  // N

inline bool is_finite(const Force3 & force)
{
  return std::isfinite(force[0]) && std::isfinite(force[1]) && std::isfinite(force[2]);
}

// 合力の大きさを max_force_n（N）以下にする。非有限なら 0 にする
inline Force3 limit_force_norm(const Force3 & force, double max_force_n)
{
  if (!is_finite(force) || !(max_force_n > 0.0)) {
    return {0.0, 0.0, 0.0};
  }
  const double norm = std::sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]);
  if (norm <= max_force_n) {
    return force;
  }
  const double scale = max_force_n / norm;
  return {force[0] * scale, force[1] * scale, force[2] * scale};
}

// 受け取った力を、受信からの経過（ns）で 0 にするかを決めて返す。
// 一度も受け取っていない（received == false）、経過が負（時計の不整合）、期限を過ぎたものは 0
inline Force3 force_within_deadline(
  const Force3 & received_force, bool received, int64_t age_ns, int64_t timeout_ns)
{
  if (!received || age_ns < 0 || age_ns > timeout_ns || !is_finite(received_force)) {
    return {0.0, 0.0, 0.0};
  }
  return received_force;
}

// ROS のコールバック（書き手）と haptic スレッド（1 kHz の読み手）の間で力を受け渡す。
// haptic スレッドを止めないよう lock を使わない。成分ごとの atomic なので、別々の受信の成分が
// 混ざることはあるが、読み手は混ざった値にも大きさの上限を掛ける
class ForceFeedbackBuffer
{
public:
  using Clock = std::chrono::steady_clock;

  void store(const Force3 & force, Clock::time_point received_at = Clock::now())
  {
    // 非有限の成分は 0 にする（以前の NaN の扱いと同じ。inf も 0 にする）
    for (size_t i = 0; i < 3; ++i) {
      force_[i].store(std::isfinite(force[i]) ? force[i] : 0.0, std::memory_order_relaxed);
    }
    received_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(received_at.time_since_epoch()).count(),
      std::memory_order_release);
  }

  // 期限内なら受け取った力、そうでなければ 0
  Force3 load(int64_t timeout_ns, Clock::time_point now = Clock::now()) const
  {
    const int64_t received_ns = received_ns_.load(std::memory_order_acquire);
    const Force3 force = {
      force_[0].load(std::memory_order_relaxed),
      force_[1].load(std::memory_order_relaxed),
      force_[2].load(std::memory_order_relaxed)};
    const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    return force_within_deadline(force, received_ns != kNeverReceived, now_ns - received_ns, timeout_ns);
  }

  // 期限内に受け取った力があるか（ログ用）
  bool fresh(int64_t timeout_ns, Clock::time_point now = Clock::now()) const
  {
    const int64_t received_ns = received_ns_.load(std::memory_order_acquire);
    const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    return received_ns != kNeverReceived && now_ns - received_ns >= 0 &&
           now_ns - received_ns <= timeout_ns;
  }

private:
  static constexpr int64_t kNeverReceived = INT64_MIN;
  std::array<std::atomic<double>, 3> force_{};
  std::atomic<int64_t> received_ns_{kNeverReceived};
};

}  // namespace touch_common

#endif  // TOUCH_COMMON__FORCE_WATCHDOG_HPP_

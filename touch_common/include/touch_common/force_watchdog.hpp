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
#include <mutex>

namespace touch_common
{

using Force3 = std::array<double, 3>;  // N

// Touch が出せる力の大きさの上限（N、データシートの最大出力）。max_force_n はこれより大きくできない
constexpr double kTouchMaxForceN = 3.3;

// max_force_n（N）として受け付けるか（有限、0 より大きく kTouchMaxForceN 以下）
inline bool valid_max_force_n(double max_force_n)
{
  return std::isfinite(max_force_n) && max_force_n > 0.0 && max_force_n <= kTouchMaxForceN;
}

// force_timeout_ms（ms）の上限。これより長い期限は、送り側が止まっても力が残る時間として長すぎる
constexpr double kMaxForceTimeoutMs = 10000.0;

// force_timeout_ms として受け付けるか（有限、0 より大きく kMaxForceTimeoutMs 以下）
inline bool valid_force_timeout_ms(double timeout_ms)
{
  return std::isfinite(timeout_ms) && timeout_ms > 0.0 && timeout_ms <= kMaxForceTimeoutMs;
}

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

// Touch に出す力（N）。期限内に受け取った力（user_force_fresh）があるときだけ、受け取った力に damping
// （−damping × 速度）を足し、合力の大きさを max_force_n 以下にする。期限切れ・未受信なら damping も含めて 0
// （送り側が止まったら、Touch は力を出さない）
inline Force3 commanded_force(
  const Force3 & user_force, bool user_force_fresh, const Force3 & velocity, double damping,
  double max_force_n)
{
  if (!user_force_fresh) {
    return {0.0, 0.0, 0.0};
  }
  return limit_force_norm(
    {user_force[0] - damping * velocity[0],
     user_force[1] - damping * velocity[1],
     user_force[2] - damping * velocity[2]},
    max_force_n);
}

// ROS のコールバック（書き手）と haptic スレッド（1 kHz の読み手）の間で、力と受信時刻を 1 組として受け渡す。
// haptic スレッドを止めないよう、読み手は lock を取らない（seqlock）。書き込みと重なった読み取りは
// kMaxReadAttempts 回まで読み直し、それでも一貫した組が取れなければ 0 とする（別々の受信の成分を混ぜない）。
class ForceFeedbackBuffer
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr int kMaxReadAttempts = 3;

  void store(const Force3 & force, Clock::time_point received_at = Clock::now())
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);  // 書き手どうしだけを直列にする
    const uint64_t seq = seq_.load(std::memory_order_relaxed);
    seq_.store(seq + 1, std::memory_order_relaxed);  // 奇数: 書き込み中
    std::atomic_thread_fence(std::memory_order_release);
    // 非有限の成分は 0 にする（以前の NaN の扱いと同じ。inf も 0 にする）
    for (size_t i = 0; i < 3; ++i) {
      force_[i].store(std::isfinite(force[i]) ? force[i] : 0.0, std::memory_order_relaxed);
    }
    received_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(received_at.time_since_epoch()).count(),
      std::memory_order_relaxed);
    seq_.store(seq + 2, std::memory_order_release);  // 偶数: 書き込み完了
  }

  // 期限内に受け取った力があれば true を返し、force にその力を入れる。なければ false と 0
  bool load(int64_t timeout_ns, Force3 & force, Clock::time_point now = Clock::now()) const
  {
    force = {0.0, 0.0, 0.0};
    Force3 received;
    int64_t received_ns = kNeverReceived;
    if (!snapshot(received, received_ns)) {
      return false;  // 書き込みと重なり続けた
    }
    if (received_ns == kNeverReceived) {
      return false;  // 未受信（INT64_MIN を引き算しない）
    }
    const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    // steady_clock の ns は単調で、受信時刻も同じ時計なので、この引き算はあふれない
    const int64_t age_ns = now_ns - received_ns;
    if (age_ns < 0 || age_ns > timeout_ns || !is_finite(received)) {
      return false;
    }
    force = received;
    return true;
  }

  // 期限内に受け取った力があるか（ログ用）
  bool fresh(int64_t timeout_ns, Clock::time_point now = Clock::now()) const
  {
    Force3 unused;
    return load(timeout_ns, unused, now);
  }

private:
  // 1 回の store() で書かれた力と受信時刻の組を読む。取れなければ false
  bool snapshot(Force3 & force, int64_t & received_ns) const
  {
    for (int attempt = 0; attempt < kMaxReadAttempts; ++attempt) {
      const uint64_t before = seq_.load(std::memory_order_acquire);
      if (before & 1u) {
        continue;  // 書き込み中
      }
      force = {
        force_[0].load(std::memory_order_relaxed),
        force_[1].load(std::memory_order_relaxed),
        force_[2].load(std::memory_order_relaxed)};
      received_ns = received_ns_.load(std::memory_order_relaxed);
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_relaxed) == before) {
        return true;
      }
    }
    return false;
  }

  static constexpr int64_t kNeverReceived = INT64_MIN;
  std::mutex writer_mutex_;
  std::atomic<uint64_t> seq_{0};
  std::array<std::atomic<double>, 3> force_{};
  std::atomic<int64_t> received_ns_{kNeverReceived};
};

}  // namespace touch_common

#endif  // TOUCH_COMMON__FORCE_WATCHDOG_HPP_

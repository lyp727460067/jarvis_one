

#ifndef NEPTURNE_COMMON_TIME_H_
#define NEPTURNE_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include <cmath>
#include "jarvis/common/port.h"
namespace jarvis {
namespace common {

constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
double ToSeconds(Duration duration);
double ToSeconds(std::chrono::steady_clock::duration duration);

// Creates a time from a Universal Time Scale.
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
std::ostream& operator<<(std::ostream& os, Time time);

// CPU time consumed by the thread so far, in seconds.
double GetThreadCpuTimeSeconds();
template <int Height = 5, int BarWidth = 1, int Padding = 1, int Offset = 0>
std::string DrawVbars(const std::vector<float>&s, const bool DrawMinMax = true) {
  std::stringstream info;
  static_assert(0 < Height and 0 < BarWidth and 0 <= Padding and 0 <= Offset);

  auto cout_n = [&](auto&& v, int n = 1) {
    while (n-- > 0) info << v;
  };
  auto lerp = [](float a, float b, float t) { return a + t * (b - a); };

  const auto [min, max] = std::minmax_element(std::cbegin(s), std::cend(s));

  std::vector<std::div_t> qr;
  for (const auto& e : s) {
    qr.push_back(std::div(lerp(0, 8 * Height, (e - *min) / (*max - *min)), 8));
  }
  for (auto h{Height}; h-- > 0; cout_n('\n')) {
    cout_n(' ', Offset);

    for (auto dv : qr) {
      const auto q{dv.quot}, r{dv.rem};
      unsigned char d[]{0xe2, 0x96, 0x88, 0};  // Full Block: '█'
      q < h ? d[0] = ' ', d[1] = 0 : q == h ? d[2] -= (7 - r) : 0;
      cout_n(d, BarWidth), cout_n(' ', Padding);
    }

    if (DrawMinMax && Height > 1)
      Height - 1 == h ? info << "┬ " << *max
      : h             ? info << "│ "
                      : info << "┴ " << *min;
  }
  return info.str();
}
}  // namespace common
}  // namespace loopdetection

#endif

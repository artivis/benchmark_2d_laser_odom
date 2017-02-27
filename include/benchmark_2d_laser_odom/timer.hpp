#ifndef _PAL_TIME_PROFILER_PAL_TIMER_HPP_
#define _PAL_TIME_PROFILER_PAL_TIMER_HPP_

//#include "timer.h"

namespace pal
{

template <typename Unit>
Timer<Unit>::Timer(): calls_(0), start_(now()), last_start_(start_) { }

template <typename Unit>
template <typename T>
T Timer<Unit>::tic()
{
  const T elapsed = static_cast<T>(cast_d(now() - last_start_).count());
  last_start_ = now();
  ++calls_;
  return elapsed;
}

template <typename Unit>
template <typename T>
T Timer<Unit>::avg_tic() const
{
  return static_cast<T>(cast_d(now() - start_).count()) /
          ((calls_ != 0)? T(calls_) : T(1));
}

template <typename Unit>
void Timer<Unit>::reset()
{
  start_ = now();
  last_start_ = start_;
  calls_ = 0;
}

template <typename Unit>
std::size_t Timer<Unit>::calls() const noexcept
{
  return calls_;
}

template <typename Unit>
template <typename... Args>
auto Timer<Unit>::cast_d(Args&&... args) const ->
decltype(std::chrono::duration_cast<Unit>(std::forward<Args>(args)...))
{
  return std::chrono::duration_cast<Unit>(std::forward<Args>(args)...);
}

template <typename Unit>
template <typename... Args>
auto Timer<Unit>::cast(Args&&... args) const ->
decltype(std::chrono::time_point_cast<Unit>(std::forward<Args>(args)...))
{
  return std::chrono::time_point_cast<Unit>(std::forward<Args>(args)...);
}

template <typename Unit>
auto Timer<Unit>::now() const ->
decltype(std::declval<Timer<Unit>>().cast(time::details::default_clock_t::now()))
{
  return cast(std::chrono::system_clock::now());
}

} /* namespace pal */

#endif /* _PAL_TIME_PROFILER_PAL_TIMER_HPP_ */

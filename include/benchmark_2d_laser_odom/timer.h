/**
 * \file timer.h
 *
 *  Created on: Feb 17, 2017
 *  \author: Jeremie Deray
 */

#ifndef _PAL_TIME_PROFILER_TIMER_H_
#define _PAL_TIME_PROFILER_TIMER_H_

#include <chrono>
#include <ctime>
#include <utility>

namespace pal
{

namespace time
{
using std::chrono::hours;
using std::chrono::minutes;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::nanoseconds;

namespace details
{
template<typename _Clock, typename _Dur>
using time_point = std::chrono::time_point<_Clock, _Dur>;

using default_clock_t = std::chrono::_V2::high_resolution_clock;
} // namespace details

} // namespace time

/**
 * @brief Timer. A tic timer.
 *
 * Mesure the elapsed time between construction - or tic() -
 * to tic(). The elapsed time is expressed in unit.
 *
 * @param unit. The time unit.
 * @see unit
 */
template <typename Unit>
class Timer
{
public:

  /**
   * @brief Timer. Launch the timer.
   */
  Timer();

  /**
   * @brief ~Timer. Default desctructor.
   */
  ~Timer() = default;

  /**
   * @brief toc. Return elapsed time since construction or last tic().
   * @return T. The elapsed time in unit.
   */
   template <typename T = int64_t>
   T tic();

   /**
    * @brief avg_toc. Return the average toc.
    * @return T. The average elapsed time in unit
    * @see tic()
    * @see unit
    */
   template <typename T = int64_t>
   T avg_tic() const;

   /**
    * @brief reset. Reset the timer.
    */
   void reset();

   /**
    * @brief getPtr
    */
   void ptr();

   /**
    * @brief calls. The number of times tic() was called.
    * @return std::size_t. The number of times tic() was called.
    */
   inline std::size_t calls() const noexcept;

protected:

  std::size_t calls_;

  time::details::time_point<time::details::default_clock_t, Unit> start_;

  time::details::time_point<time::details::default_clock_t, Unit> last_start_;

  template <typename... Args>
  auto cast_d(Args&&... args) const ->
    decltype(std::chrono::duration_cast<Unit>(std::forward<Args>(args)...));

  template <typename... Args>
  auto cast(Args&&... args) const ->
    decltype(std::chrono::time_point_cast<Unit>(std::forward<Args>(args)...));

  auto now() const ->
  decltype(std::declval<Timer<Unit>>().cast(time::details::default_clock_t::now()));
};

using TimerS = Timer<time::seconds>;
using TimerM = Timer<time::milliseconds>;
using TimerU = Timer<time::microseconds>;
using TimerN = Timer<time::nanoseconds>;

} /* namespace pal */

#include "timer.hpp"

#endif /* _PAL_TIME_PROFILER_TIMER_H_ */

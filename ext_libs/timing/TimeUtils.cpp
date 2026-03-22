#include "TimeUtils.h"

#include <chrono>

int64_t getSteadyTimeByMs()
{
    std::chrono::steady_clock::time_point tnow = std::chrono::steady_clock::now();
    std::chrono::milliseconds ttimeMillis = std::chrono::duration_cast<std::chrono::milliseconds>(tnow.time_since_epoch()); // covert to milliseconds.
    return static_cast<int64_t>(ttimeMillis.count());
}

int64_t getCurrentTimeByMs()
{
    std::chrono::system_clock::time_point tnow = std::chrono::system_clock::now();  // get currernt time.
    std::chrono::milliseconds ttimeMillis = std::chrono::duration_cast<std::chrono::milliseconds>(tnow.time_since_epoch()); // covert to milliseconds.
    return static_cast<int64_t>(ttimeMillis.count());
}
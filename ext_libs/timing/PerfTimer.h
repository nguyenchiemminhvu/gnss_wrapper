/**
 * @file        PerfTimer.h
 * @brief       Timer to check the performance in nano, micro, mili seconds...
 * @author      vu3.nguyen
 * @version     0.01.001 (A: major change, B: refactoring or structure change, C: minor change (ex) 1.12.133)
 * @usage
 * PerfTimer* timer = GetPerfTimer();
 * timer->start(MeasurementType::NAV_PVAT_TRANSFER_DELAY_MEASUREMENT);
 * // do something here
 * timer->stop();
 * 
 * or
 * timer->record(MeasurementType::NAV_PVT_SENDING);
 */

#ifndef PERF_TIMER_H
#define PERF_TIMER_H

#include "singleton.h"

#include <chrono>
#include <string>
#include <vector>
#include <deque>

#define PERF_TIMER_TABLE_MAX_SIZE (2U)
#define NANOSECONDS_TO_MILLISECONDS (1e-6)

class PerfTimer
{
public:
    using clock = std::chrono::steady_clock;
    using clock_t = std::chrono::time_point<std::chrono::steady_clock>;
    using duration_t = std::chrono::time_point<std::chrono::steady_clock>::duration;

    enum MeasurementType : std::size_t
    {
        NAV_PVT_SENDING,
        NAV_PVT_DETECTED,
        UART_READING,
        UBX_PROCESS_AN_EPOCH,

        MEASUREMENT_TYPE_TOTAL
    };

    PerfTimer();
    virtual ~PerfTimer();

    void record(std::size_t pt_id);
    void start(std::size_t pt_id);
    void stop(std::size_t pt_id);

private:
    std::vector<std::deque<clock_t>> m_timeTable;

private:
    void measureDuration(std::size_t pt_id);
};

// macro to map MeasurementType enum values to their corresponding names
#define MEASUREMENT_TYPE_TO_STRING(type) \
    (((type) == static_cast<std::size_t>(NAV_PVT_SENDING)) ? "NAV_PVT_SENDING" : \
     ((type) == static_cast<std::size_t>(NAV_PVT_DETECTED)) ? "NAV_PVT_DETECTED" : \
     ((type) == static_cast<std::size_t>(UART_READING)) ? "UART_READING" : \
     ((type) == static_cast<std::size_t>(UBX_PROCESS_AN_EPOCH)) ? "UBX_PROCESS_AN_EPOCH" : "UNKNOWN")

#define GetPerfTimer() UniqueInstance<PerfTimer>::getInstance()

#endif // PERF_TIMER_H
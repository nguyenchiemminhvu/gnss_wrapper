#include "PerfTimer.h"

#include "logging.h"

PerfTimer::PerfTimer()
{
    m_timeTable.resize(MeasurementType::MEASUREMENT_TYPE_TOTAL);
    for (std::size_t i = 0U; i < m_timeTable.size(); i++)
    {
        m_timeTable[i].resize(PERF_TIMER_TABLE_MAX_SIZE);
    }
}

PerfTimer::~PerfTimer()
{

}

void PerfTimer::record(std::size_t pt_id)
{
    if (pt_id < static_cast<std::size_t>(MeasurementType::MEASUREMENT_TYPE_TOTAL))
    {
        std::deque<clock_t>& row = m_timeTable[pt_id];
        row[0] = row[1];
        row[1] = std::chrono::steady_clock::now();

        measureDuration(pt_id);
    }
}

void PerfTimer::start(std::size_t pt_id)
{
    if (pt_id < static_cast<std::size_t>(MeasurementType::MEASUREMENT_TYPE_TOTAL))
    {
        std::deque<clock_t>& row = m_timeTable[pt_id];
        row[0] = std::chrono::steady_clock::now();
    }
}

void PerfTimer::stop(std::size_t pt_id)
{
    if (pt_id < static_cast<std::size_t>(MeasurementType::MEASUREMENT_TYPE_TOTAL))
    {
        std::deque<clock_t>& row = m_timeTable[pt_id];
        row[1] = std::chrono::steady_clock::now();

        measureDuration(pt_id);
    }
}

void PerfTimer::measureDuration(std::size_t pt_id)
{
    if (pt_id < static_cast<std::size_t>(MeasurementType::MEASUREMENT_TYPE_TOTAL))
    {
        const std::deque<clock_t>& row = m_timeTable[pt_id];
        duration_t d = row[1] - row[0];
        double dMs = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(d).count()) * NANOSECONDS_TO_MILLISECONDS;

        LOG_INFO("[PerfTimer] ", MEASUREMENT_TYPE_TO_STRING(pt_id), ": ", dMs, " (ms)");
    }
}

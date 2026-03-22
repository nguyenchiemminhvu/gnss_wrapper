/**
 * MIT License
 *
 * Copyright (c) 2026 nguyenchiemminhvu@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file cpu_utils.h
 * @brief CPU topology, usage monitoring, affinity, and scheduling utilities.
 *
 * This library provides a C-compatible API (usable from C++) for:
 *
 *  - Querying the number of online/configured CPU cores.
 *  - Taking point-in-time snapshots of per-core CPU time counters read from
 *    /proc/stat, and computing per-core utilisation between two snapshots.
 *  - Convenience wrappers that measure utilisation over a caller-specified
 *    sampling interval and identify the least- or most-loaded core.
 *  - Setting and querying CPU affinity masks for threads and processes via
 *    pthread_setaffinity_np / sched_setaffinity.
 *  - Setting and querying Linux scheduling policies (SCHED_OTHER, SCHED_FIFO,
 *    SCHED_RR, etc.) and nice values for threads and processes.
 *  - cpu_set_t helper utilities (zero, set, test, count, stringify).
 *
 * Thread safety:
 *  - Each thread has its own error buffer (thread-local storage), so
 *    last_error_string() is always safe to call from multiple threads.
 *  - All other functions are stateless (no shared mutable global state)
 *    and are therefore safe to call concurrently, provided the caller
 *    passes distinct snapshot_t objects.
 *
 * Error handling:
 *  - All functions that can fail return 0 on success and -1 on failure.
 *  - On failure the human-readable reason can be retrieved with
 *    last_error_string(), which reads from a thread-local buffer.
 *
 * Example usage:
 *
 * @code
 *   using namespace cpu_utils;
 *
 *   int main(void)
 *   {
 *       int cpu_id;
 *       double usage;
 *
 *       // Find the least-loaded core by sampling for 500 ms.
 *       if (find_least_used_core(500, &cpu_id, &usage) != 0)
 *       {
 *           fprintf(stderr, "find least-used core failed: %s\n", last_error_string());
 *           return 1;
 *       }
 *
 *       printf("Least-used core: %d (%.2f%%)\n", cpu_id, usage);
 *
 *       // Pin the calling thread to that core.
 *       if (thread_bind_self_to_cpu(cpu_id) != 0)
 *       {
 *           fprintf(stderr, "bind self failed: %s\n", last_error_string());
 *           return 1;
 *       }
 *
 *       // Also pin the whole process to the same core.
 *       if (process_bind_to_cpu(getpid(), cpu_id) != 0)
 *       {
 *           fprintf(stderr, "bind process failed: %s\n", last_error_string());
 *           return 1;
 *       }
 *
 *       return 0;
 *   }
 * @endcode
 */

#ifndef CPU_UTILS_H
#define CPU_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <pthread.h>
#include <sched.h>
#include <sys/types.h>
#include <stddef.h>

/**
 * @brief Maximum length (including NUL) for name strings used internally.
 *
 * Callers may override this at compile time by defining CPU_UTILS_MAX_NAME
 * before including this header.
 */
#ifndef CPU_UTILS_MAX_NAME
#define CPU_UTILS_MAX_NAME 64
#endif

namespace cpu_utils
{

/**
 * @brief Raw CPU-time counters for a single logical core.
 *
 * These values are read directly from a "cpu<N>" line in /proc/stat and
 * represent cumulative jiffies spent in each mode since boot.  The kernel
 * documentation defines the fields as follows:
 *
 *  - user    : Time in user mode (normal processes).
 *  - nice    : Time in user mode with lowered scheduling priority (niced).
 *  - system  : Time in kernel (system call) mode.
 *  - idle    : Time spent idle (doing nothing).
 *  - iowait  : Time waiting for I/O to complete (unreliable on SMP kernels,
 *              but still useful for relative comparisons).
 *  - irq     : Time servicing hardware interrupts.
 *  - softirq : Time servicing software interrupts.
 *  - steal   : Time "stolen" by a hypervisor servicing another virtual CPU.
 *  - total   : Sum of all of the above fields; used as the denominator when
 *              computing utilisation percentages.
 */
typedef struct core_times_s
{
    unsigned long long user;     /**< Jiffies in user mode. */
    unsigned long long nice;     /**< Jiffies in user mode (niced). */
    unsigned long long system;   /**< Jiffies in kernel mode. */
    unsigned long long idle;     /**< Jiffies spent idle. */
    unsigned long long iowait;   /**< Jiffies waiting for I/O. */
    unsigned long long irq;      /**< Jiffies servicing hardware IRQs. */
    unsigned long long softirq;  /**< Jiffies servicing software IRQs. */
    unsigned long long steal;    /**< Jiffies stolen by hypervisor. */
    unsigned long long total;    /**< Sum of all counters above. */
} core_times_t;

/**
 * @brief Per-core CPU utilisation result.
 *
 * Produced by get_usage_between() after comparing two snapshots.
 * The usage_percent value expresses what fraction of total CPU time was
 * spent in non-idle states during the interval between the two snapshots.
 */
typedef struct core_usage_s
{
    int    core_id;                  /**< Logical core index (0-based). */
    double usage_percent;            /**< Utilisation in the range [0.0, 100.0]. */
    unsigned long long total_diff;   /**< Change in total jiffies over the interval. */
    unsigned long long idle_diff;    /**< Change in idle+iowait jiffies over the interval. */
} core_usage_t;

/**
 * @brief A point-in-time snapshot of per-core CPU time counters.
 *
 * Allocate with snapshot_init(), populate with snapshot_take(), and release
 * with snapshot_free().  Two snapshots bracketing a time window can be passed
 * to get_usage_between() to compute per-core utilisation for that window.
 */
typedef struct snapshot_s
{
    int           cpu_count; /**< Number of logical cores in the @c cores array. */
    core_times_t *cores;     /**< Heap-allocated array of per-core time counters. */
} snapshot_t;

/* -------------------------------------------------------------------------
 * Snapshot lifecycle
 * ---------------------------------------------------------------------- */

/**
 * @brief Allocate and zero-initialise a snapshot for @p cpu_count cores.
 *
 * The @c cores array inside @p snap is allocated on the heap. The caller
 * is responsible for releasing it with snapshot_free() when done.
 *
 * @param snap      Non-NULL pointer to an uninitialised snapshot_t.
 * @param cpu_count Number of logical cores to allocate space for (> 0).
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int snapshot_init(snapshot_t *snap, int cpu_count);

/**
 * @brief Release heap memory owned by @p snap and reset its fields to zero.
 *
 * Safe to call with a NULL pointer (no-op).
 *
 * @param snap Pointer to the snapshot to free.
 */
void snapshot_free(snapshot_t *snap);

/**
 * @brief Populate @p snap with the current per-core CPU time counters.
 *
 * Reads /proc/stat and fills each entry in @p snap->cores with the latest
 * cumulative jiffie counters.  @p snap must already be initialised via
 * snapshot_init().
 *
 * @param snap Non-NULL, previously initialised snapshot_t.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int snapshot_take(snapshot_t *snap);

/* -------------------------------------------------------------------------
 * CPU count / topology
 * ---------------------------------------------------------------------- */

/**
 * @brief Return the number of logical CPU cores currently online.
 *
 * Wraps sysconf(_SC_NPROCESSORS_ONLN).  "Online" means the core is
 * present and available to the scheduler.
 *
 * @return Number of online cores (>= 1), or -1 on error.
 */
int get_online_cpu_count(void);

/**
 * @brief Return the number of logical CPU cores present in the system.
 *
 * Wraps sysconf(_SC_NPROCESSORS_CONF).  This counts all configured cores,
 * including those that may be offline (hotplugged out).
 *
 * @return Total number of configured cores (>= 1), or -1 on error.
 */
int get_configured_cpu_count(void);

/**
 * @brief Return the index of the logical CPU core currently executing the
 *        calling thread.
 *
 * Wraps sched_getcpu(3).  Because the scheduler may migrate the thread at
 * any time, the value is only a best-effort snapshot.
 *
 * @return Core index (>= 0), or -1 on error.
 */
int get_current_cpu(void);

/* -------------------------------------------------------------------------
 * Usage
 * ---------------------------------------------------------------------- */

/**
 * @brief Compute per-core utilisation between two snapshots.
 *
 * For each logical core i the function computes:
 * @code
 *   idle_diff  = (b->cores[i].idle + b->cores[i].iowait)
 *              - (a->cores[i].idle + a->cores[i].iowait)
 *   total_diff = b->cores[i].total - a->cores[i].total
 *   usage      = 100.0 * (1.0 - idle_diff / total_diff)   ; clamped to [0,100]
 * @endcode
 *
 * Both snapshots must have been created for the same cpu_count.
 * @p out_array must have room for at least @p a->cpu_count entries.
 *
 * @param a         Earlier snapshot.
 * @param b         Later snapshot.
 * @param out_array Caller-allocated array receiving one core_usage_t per core.
 * @param out_count Capacity of @p out_array (must be >= a->cpu_count).
 * @return Number of entries written (== a->cpu_count), or -1 on error.
 */
int get_usage_between(const snapshot_t *a,
                      const snapshot_t *b,
                      core_usage_t *out_array,
                      int out_count);

/**
 * @brief Measure the utilisation of a single core over a sampling interval.
 *
 * Takes two snapshots separated by @p interval_ms milliseconds (or 1 second
 * if interval_ms is 0) and returns the utilisation of @p core_id via
 * @p usage_percent.
 *
 * This function blocks the calling thread for the duration of the interval.
 *
 * @param core_id       0-based index of the core to measure.
 * @param interval_ms   Sampling window in milliseconds; 0 means 1000 ms.
 * @param usage_percent Output pointer receiving the utilisation in [0,100].
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int get_core_usage_percent(int core_id,
                           unsigned int interval_ms,
                           double *usage_percent);

/**
 * @brief Measure the utilisation of all online cores over a sampling interval.
 *
 * Equivalent to calling get_core_usage_percent() for every online core in a
 * single pass.  Blocks for @p interval_ms milliseconds.
 *
 * @param interval_ms Sampling window in milliseconds; 0 means 1000 ms.
 * @param out_array   Caller-allocated array; must hold at least
 *                    get_online_cpu_count() entries.
 * @param out_count   Capacity of @p out_array.
 * @return Number of entries written on success, -1 on error.
 */
int get_all_core_usage(unsigned int interval_ms,
                       core_usage_t *out_array,
                       int out_count);

/**
 * @brief Identify the least-loaded logical core over a sampling interval.
 *
 * Measures all cores and selects the one with the lowest utilisation.
 * Blocks for @p interval_ms milliseconds.
 *
 * @param interval_ms   Sampling window in milliseconds; 0 means 1000 ms.
 * @param core_id       Optional output pointer receiving the core index.
 * @param usage_percent Optional output pointer receiving the utilisation.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int find_least_used_core(unsigned int interval_ms,
                         int *core_id,
                         double *usage_percent);

/**
 * @brief Identify the most-loaded logical core over a sampling interval.
 *
 * Measures all cores and selects the one with the highest utilisation.
 * Blocks for @p interval_ms milliseconds.
 *
 * @param interval_ms   Sampling window in milliseconds; 0 means 1000 ms.
 * @param core_id       Optional output pointer receiving the core index.
 * @param usage_percent Optional output pointer receiving the utilisation.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int find_most_used_core(unsigned int interval_ms,
                        int *core_id,
                        double *usage_percent);

/* -------------------------------------------------------------------------
 * Thread affinity
 * ---------------------------------------------------------------------- */

/**
 * @brief Pin @p thread to a single logical core identified by @p cpu_id.
 *
 * Builds a cpu_set_t containing only @p cpu_id and passes it to
 * thread_set_affinity().
 *
 * @param thread  POSIX thread handle.
 * @param cpu_id  0-based core index; must be in [0, online_count).
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int thread_bind_to_cpu(pthread_t thread, int cpu_id);

/**
 * @brief Pin the calling thread to a single logical core.
 *
 * Convenience wrapper around thread_bind_to_cpu(pthread_self(), cpu_id).
 *
 * @param cpu_id 0-based core index.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int thread_bind_self_to_cpu(int cpu_id);

/**
 * @brief Measure all cores, then pin the calling thread to the least-loaded one.
 *
 * Combines find_least_used_core() and thread_bind_self_to_cpu() in a single
 * call.  Blocks for @p interval_ms milliseconds during the measurement phase.
 *
 * @param interval_ms   Sampling window in milliseconds; 0 means 1000 ms.
 * @param selected_cpu  Optional output pointer receiving the chosen core index.
 * @param usage_percent Optional output pointer receiving the utilisation of the
 *                      chosen core.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int thread_bind_self_to_least_used_cpu(unsigned int interval_ms,
                                       int *selected_cpu,
                                       double *usage_percent);

/**
 * @brief Retrieve the CPU affinity mask of @p thread.
 *
 * Wraps pthread_getaffinity_np(3).  The set is zeroed before the call so
 * that bits for unavailable cores are cleared.
 *
 * @param thread POSIX thread handle.
 * @param set    Output cpu_set_t to fill; must not be NULL.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int thread_get_affinity(pthread_t thread, cpu_set_t *set);

/**
 * @brief Apply the CPU affinity mask @p set to @p thread.
 *
 * Wraps pthread_setaffinity_np(3).
 *
 * @param thread POSIX thread handle.
 * @param set    Non-NULL cpu_set_t describing the allowed core(s).
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int thread_set_affinity(pthread_t thread, const cpu_set_t *set);

/* -------------------------------------------------------------------------
 * Process affinity
 * ---------------------------------------------------------------------- */

/**
 * @brief Pin process @p pid to a single logical core identified by @p cpu_id.
 *
 * Builds a cpu_set_t containing only @p cpu_id and passes it to
 * process_set_affinity().
 *
 * @param pid    Process ID; use 0 for the calling process.
 * @param cpu_id 0-based core index; must be in [0, online_count).
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int process_bind_to_cpu(pid_t pid, int cpu_id);

/**
 * @brief Retrieve the CPU affinity mask of process @p pid.
 *
 * Wraps sched_getaffinity(2).  The set is zeroed before the call.
 *
 * @param pid Process ID; use 0 for the calling process.
 * @param set Output cpu_set_t; must not be NULL.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int process_get_affinity(pid_t pid, cpu_set_t *set);

/**
 * @brief Apply the CPU affinity mask @p set to process @p pid.
 *
 * Wraps sched_setaffinity(2).  This affects all threads of the target
 * process.
 *
 * @param pid Process ID; use 0 for the calling process.
 * @param set Non-NULL cpu_set_t describing the allowed core(s).
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int process_set_affinity(pid_t pid, const cpu_set_t *set);

/* -------------------------------------------------------------------------
 * CPU set helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief Clear all bits in @p set (equivalent to CPU_ZERO).
 *
 * Safe to call with a NULL pointer (no-op).
 *
 * @param set cpu_set_t to clear.
 */
void cpuset_zero(cpu_set_t *set);

/**
 * @brief Set the bit for @p cpu_id in @p set (equivalent to CPU_SET).
 *
 * Safe to call with a NULL pointer or a negative @p cpu_id (no-op).
 *
 * @param set    cpu_set_t to modify.
 * @param cpu_id 0-based core index to enable.
 */
void cpuset_set(cpu_set_t *set, int cpu_id);

/**
 * @brief Test whether @p cpu_id is set in @p set.
 *
 * @param set    cpu_set_t to query.
 * @param cpu_id 0-based core index to test.
 * @return 1 if the bit is set, 0 otherwise (also 0 for NULL or negative id).
 */
int cpuset_isset(const cpu_set_t *set, int cpu_id);

/**
 * @brief Return the number of bits set in @p set (equivalent to CPU_COUNT).
 *
 * @param set cpu_set_t to count.
 * @return Number of set bits, or 0 for a NULL pointer.
 */
int cpuset_count(const cpu_set_t *set);

/**
 * @brief Format the set bits of @p set as a comma-separated string.
 *
 * Example output: "0,2,3" for a set containing cores 0, 2, and 3.
 *
 * @param set         cpu_set_t to format.
 * @param buffer      Caller-allocated output buffer.
 * @param buffer_size Size of @p buffer in bytes (must be > 0).
 * @return 0 on success, -1 if the buffer is too small or arguments are invalid.
 */
int cpuset_to_string(const cpu_set_t *set,
                     char *buffer,
                     size_t buffer_size);

/* -------------------------------------------------------------------------
 * Scheduling / priority helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief Set the scheduling policy and real-time priority of @p thread.
 *
 * Wraps pthread_setschedparam(3).  Valid policies are SCHED_OTHER (0 priority),
 * SCHED_FIFO, and SCHED_RR (priority range: sched_get_priority_min/max).
 *
 * @param thread   POSIX thread handle.
 * @param policy   Scheduling policy (e.g. SCHED_FIFO, SCHED_RR, SCHED_OTHER).
 * @param priority Real-time priority; must be 0 for SCHED_OTHER/SCHED_BATCH/SCHED_IDLE.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int thread_set_sched_policy(pthread_t thread, int policy, int priority);

/**
 * @brief Set the scheduling policy and priority of the calling thread.
 *
 * Convenience wrapper around thread_set_sched_policy(pthread_self(), ...).
 *
 * @param policy   Scheduling policy.
 * @param priority Real-time priority.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int thread_set_self_sched_policy(int policy, int priority);

/**
 * @brief Retrieve the current scheduling policy and priority of @p thread.
 *
 * Wraps pthread_getschedparam(3).
 *
 * @param thread   POSIX thread handle.
 * @param policy   Output pointer receiving the scheduling policy.
 * @param priority Output pointer receiving the real-time priority.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int thread_get_sched_policy(pthread_t thread, int *policy, int *priority);

/**
 * @brief Set the nice value (time-sharing bias) of the calling thread.
 *
 * Uses setpriority(PRIO_PROCESS, gettid(), nice_value) with the kernel thread
 * ID so that only the calling thread is affected, not the whole process.
 * Valid range is typically [-20, 19]; a higher value means lower priority.
 *
 * @note Decreasing nice below its current value requires CAP_SYS_NICE.
 *
 * @param nice_value Nice increment in [-20, 19].
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int set_current_thread_nice(int nice_value);

/**
 * @brief Set the scheduling policy and real-time priority of process @p pid.
 *
 * Wraps sched_setscheduler(2).  Affects all threads of the process.
 *
 * @param pid      Target process ID; use 0 for the calling process.
 * @param policy   Scheduling policy.
 * @param priority Real-time priority.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int process_set_sched_policy(pid_t pid, int policy, int priority);

/**
 * @brief Set the scheduling policy and priority of the calling process.
 *
 * Convenience wrapper around process_set_sched_policy(getpid(), ...).
 *
 * @param policy   Scheduling policy.
 * @param priority Real-time priority.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int process_set_self_sched_policy(int policy, int priority);

/**
 * @brief Retrieve the scheduling policy and priority of process @p pid.
 *
 * Combines sched_getscheduler(2) and sched_getparam(2).
 *
 * @param pid      Target process ID; use 0 for the calling process.
 * @param policy   Output pointer receiving the scheduling policy.
 * @param priority Output pointer receiving the real-time priority.
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int process_get_sched_policy(pid_t pid, int *policy, int *priority);

/**
 * @brief Set the nice value of the calling process.
 *
 * Uses setpriority(PRIO_PROCESS, getpid(), nice_value), which affects all
 * threads that have not set a per-thread nice value.  Valid range is typically
 * [-20, 19]; a higher value means lower priority.
 *
 * @note Decreasing nice below its current value requires CAP_SYS_NICE.
 *
 * @param nice_value Nice increment in [-20, 19].
 * @return 0 on success, -1 on error (see last_error_string()).
 */
int set_current_process_nice(int nice_value);

/* -------------------------------------------------------------------------
 * Informational helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief Convert a scheduling policy integer to a human-readable string.
 *
 * Recognised values: SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO,
 * SCHED_RR.  Any unrecognised value returns "UNKNOWN".
 *
 * @param policy Scheduling policy constant.
 * @return Pointer to a string literal (never NULL; no need to free).
 */
const char *policy_to_string(int policy);

/**
 * @brief Return a description of the last error that occurred in the calling
 *        thread, or "no error" if no error has been recorded.
 *
 * The string is stored in thread-local storage and is valid until the next
 * cpu_utils call that sets an error on the same thread.
 *
 * @return Pointer to a thread-local NUL-terminated error string (never NULL).
 */
const char *last_error_string(void);

} // namespace cpu_utils

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CPU_UTILS_H
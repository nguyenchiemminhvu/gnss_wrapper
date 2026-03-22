/**
 * @file cpu_utils.cpp
 * @brief Implementation of the cpu_utils library.
 *
 * This file implements CPU topology queries, per-core usage monitoring,
 * CPU affinity control for threads and processes, and scheduling
 * policy / nice-value helpers for Linux.
 *
 * -------------------------------------------------------------------------
 * CPU Usage Calculation - Overview
 * -------------------------------------------------------------------------
 *
 * The Linux kernel exposes cumulative per-core CPU time counters in
 * /proc/stat.  Each "cpu<N>" line contains jiffie counts broken down
 * by mode (user, nice, system, idle, iowait, irq, softirq, steal).
 *
 * To measure utilisation over a time window [t0, t1], the library takes
 * two snapshots and computes deltas for each counter:
 *
 *   total_diff = total(t1) - total(t0)
 *
 * where:
 *   total = user + nice + system + idle + iowait + irq + softirq + steal
 *
 * The combined "idle" time (i.e. time the core was NOT doing useful work)
 * is the sum of the idle and iowait fields:
 *
 *   idle_diff = (idle(t1) + iowait(t1)) - (idle(t0) + iowait(t0))
 *
 * CPU utilisation percentage is then:
 *
 *              ( idle_diff )
 *   usage = 1 - -----------  * 100
 *              ( total_diff)
 *
 * The result is clamped to [0.0, 100.0] to absorb any counter wrap-around
 * artefacts or transient negative deltas.
 *
 * -------------------------------------------------------------------------
 * Design Notes
 * -------------------------------------------------------------------------
 *
 *  - Error state is stored in a thread-local buffer (g_errbuf).  The static
 *    helpers set_error() and set_error_errno() always return -1, so callers
 *    can write:  return set_error("message");
 *
 *  - All heap allocations use the C allocator (calloc/free) so the library
 *    can be linked into both C and C++ translation units without pulling in
 *    the full C++ runtime.
 *
 *  - Sampling functions block via usleep() for the requested interval.
 *    Callers that need non-blocking behaviour should run them on a dedicated
 *    background thread.
 *
 *  - A do-while(0) block is used as a structured goto: any failure breaks
 *    out while still reaching the cleanup code at the bottom of the function.
 */

#include "cpu_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>

namespace cpu_utils
{

/* -------------------------------------------------------------------------
 * Thread-local error state
 * ---------------------------------------------------------------------- */

/**
 * Thread-local buffer that holds the last error message produced on the
 * calling thread.  Using __thread (GCC/Clang TLS extension) ensures that
 * concurrent calls from different threads never clobber each other's error.
 */
static __thread char g_errbuf[256];

/**
 * @brief Store a plain error message in the thread-local buffer and return -1.
 *
 * Passing NULL leaves the existing message untouched (no-op for the copy,
 * but still returns -1 so the idiom  "return set_error(NULL);"  works).
 *
 * @param msg  Human-readable error description, or NULL.
 * @return Always -1, enabling: return set_error("...");
 */
static int set_error(const char *msg)
{
    if (msg)
    {
        snprintf(g_errbuf, sizeof(g_errbuf), "%s", msg);
    }
    return -1;
}

/**
 * @brief Store "msg: strerror(err)" in the thread-local buffer and return -1.
 *
 * Combines a caller-supplied context string with the POSIX description of
 * the given errno value.  Used immediately after a failing syscall while
 * errno is still valid.
 *
 * @param msg  Context string (e.g. the name of the failing syscall).
 * @param err  errno value to describe.
 * @return Always -1.
 */
static int set_error_errno(const char *msg, int err)
{
    snprintf(g_errbuf, sizeof(g_errbuf), "%s: %s", msg, strerror(err));
    return -1;
}

/* -------------------------------------------------------------------------
 * Public: error retrieval
 * ---------------------------------------------------------------------- */

const char *last_error_string(void)
{
    /* Return a non-NULL sentinel when no error has been recorded yet so that
     * callers can always safely pass the result to printf/fprintf. */
    return g_errbuf[0] ? g_errbuf : "no error";
}

/* -------------------------------------------------------------------------
 * Public: CPU count / topology
 * ---------------------------------------------------------------------- */

int get_online_cpu_count(void)
{
    /* _SC_NPROCESSORS_ONLN counts only cores currently available to the
     * scheduler.  Cores that have been hot-unplugged are excluded. */
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    if (n <= 0)
    {
        set_error("sysconf(_SC_NPROCESSORS_ONLN) failed");
        return -1;
    }
    return (int)n;
}

int get_configured_cpu_count(void)
{
    /* _SC_NPROCESSORS_CONF counts all cores present in the hardware
     * description, whether they are online or not. */
    long n = sysconf(_SC_NPROCESSORS_CONF);
    if (n <= 0)
    {
        set_error("sysconf(_SC_NPROCESSORS_CONF) failed");
        return -1;
    }
    return (int)n;
}

int get_current_cpu(void)
{
    /* sched_getcpu(3) returns the index of the core the calling thread is
     * currently executing on.  The value is a best-effort snapshot; the
     * scheduler may migrate the thread before the caller can act on it. */
    int cpu = sched_getcpu();
    if (cpu < 0)
    {
        return set_error_errno("sched_getcpu failed", errno);
    }
    return cpu;
}

/* -------------------------------------------------------------------------
 * Public: snapshot lifecycle
 * ---------------------------------------------------------------------- */

int snapshot_init(snapshot_t *snap, int cpu_count)
{
    if (!snap || cpu_count <= 0)
    {
        return set_error("invalid arguments to snapshot_init");
    }

    /* calloc zeroes the memory, so all jiffy fields start at 0.  This
     * prevents a subtraction from a freshly-taken snapshot producing a
     * spurious negative delta on the first measurement. */
    snap->cores = (core_times_t *)calloc((size_t)cpu_count, sizeof(core_times_t));
    if (!snap->cores)
    {
        return set_error_errno("calloc failed", errno);
    }

    snap->cpu_count = cpu_count;
    return 0;
}

void snapshot_free(snapshot_t *snap)
{
    if (!snap)
    {
        return; /* NULL-safe: nothing to do. */
    }

    /* Zero out the struct after freeing so a stale pointer cannot be
     * accidentally dereferenced after the snapshot has been released. */
    free(snap->cores);
    snap->cores = NULL;
    snap->cpu_count = 0;
}

/* -------------------------------------------------------------------------
 * Internal: /proc/stat parsing
 * ---------------------------------------------------------------------- */

/**
 * @brief Parse one per-core "cpu<N> ..." line from /proc/stat.
 *
 * The kernel writes one aggregate line ("cpu  ...") and one line per
 * logical core ("cpu0 ...", "cpu1 ...", etc.).  Each line has the form:
 *
 *   cpu<N>  user  nice  system  idle  iowait  irq  softirq  steal
 *           [guest  guest_nice]   <- kernel >= 2.6.33, ignored here
 *
 * All values are cumulative jiffies since boot.  Guest time is
 * intentionally skipped: on modern kernels it is already included in
 * the 'user' counter, so adding it again would double-count it.
 *
 * The function requires at least 5 successfully parsed tokens
 * (cpu_id + user + nice + system + idle).  Fields absent on older
 * kernels (iowait, irq, softirq, steal) remain 0 thanks to the
 * memset performed before sscanf.
 *
 * After parsing, the function pre-computes:
 *
 *   total = user + nice + system + idle + iowait + irq + softirq + steal
 *
 * so callers can compute utilisation with a single subtraction.
 *
 * @param line    NUL-terminated /proc/stat line starting with "cpu<N>".
 * @param core_id Output: the parsed logical core index N.
 * @param t       Output: parsed counters plus their pre-computed sum.
 * @return 0 on success, -1 if the line does not have enough valid fields.
 */
static int parse_core_line(const char *line, int *core_id, core_times_t *t)
{
    int cpu_id = -1;
    core_times_t tmp;
    memset(&tmp, 0, sizeof(tmp)); /* initialise to 0 for missing fields */

    int n = sscanf(line,
                   "cpu%d %llu %llu %llu %llu %llu %llu %llu %llu",
                   &cpu_id,
                   &tmp.user, &tmp.nice, &tmp.system, &tmp.idle,
                   &tmp.iowait, &tmp.irq, &tmp.softirq, &tmp.steal);

    if (n < 5)
    {
        return -1; /* not enough fields to be useful */
    }

    /* Pre-compute total so the usage formula only needs one subtraction. */
    tmp.total = tmp.user + tmp.nice + tmp.system + tmp.idle +
                tmp.iowait + tmp.irq + tmp.softirq + tmp.steal;

    *core_id = cpu_id;
    *t = tmp;
    return 0;
}

/* -------------------------------------------------------------------------
 * Public: snapshot population
 * ---------------------------------------------------------------------- */

int snapshot_take(snapshot_t *snap)
{
    if (!snap || !snap->cores || snap->cpu_count <= 0)
    {
        return set_error("invalid snapshot in snapshot_take");
    }

    /* /proc/stat is updated by the kernel on every timer tick (typically
     * every 1-10 ms depending on CONFIG_HZ).  Opening it is cheap because
     * it is a virtual file backed by in-kernel counters. */
    FILE *fp = fopen("/proc/stat", "r");
    if (!fp)
    {
        return set_error_errno("fopen(/proc/stat) failed", errno);
    }

    char line[512];
    while (fgets(line, sizeof(line), fp))
    {
        int core_id = -1;
        core_times_t t;

        /* Skip lines that do not start with "cpu". */
        if (strncmp(line, "cpu", 3) != 0)
        {
            continue;
        }

        /* The aggregate summary line looks like "cpu  <totals>".
         * Per-core lines start with a digit: "cpu0", "cpu1", etc.
         * Skip the aggregate line so only individual core data is stored. */
        if (line[3] < '0' || line[3] > '9')
        {
            continue; /* skip aggregate "cpu " line */
        }

        if (parse_core_line(line, &core_id, &t) == 0)
        {
            /* Guard against a kernel reporting more cores than the snapshot
             * was initialised for (e.g. after CPU hotplug). */
            if (core_id >= 0 && core_id < snap->cpu_count)
            {
                snap->cores[core_id] = t;
            }
        }
    }

    fclose(fp);
    return 0;
}

/* -------------------------------------------------------------------------
 * Public: usage computation
 * ---------------------------------------------------------------------- */

/**
 * CPU utilisation formula (applied per core):
 *
 *   Let A = snapshot taken at time t0
 *       B = snapshot taken at time t1  (t1 > t0)
 *
 *   total_diff = B.total - A.total
 *             = (B.user + B.nice + B.system + B.idle + B.iowait
 *                + B.irq + B.softirq + B.steal)
 *             - (A.user + A.nice + A.system + A.idle + A.iowait
 *                + A.irq + A.softirq + A.steal)
 *
 *   idle_diff  = (B.idle + B.iowait) - (A.idle + A.iowait)
 *
 *   usage_percent = (1 - idle_diff / total_diff) * 100
 *                 = ((total_diff - idle_diff) / total_diff) * 100
 *
 * The result is clamped to [0.0, 100.0] to handle any counter wrap-around
 * or sub-millisecond sampling artefacts.  If total_diff == 0 (the interval
 * was too short for any tick to occur) the usage is reported as 0.0 %.
 */
int get_usage_between(const snapshot_t *a,
                           const snapshot_t *b,
                           core_usage_t *out_array,
                           int out_count)
{
    if (!a || !b || !out_array || !a->cores || !b->cores)
    {
        return set_error("invalid arguments to get_usage_between");
    }

    if (a->cpu_count <= 0 || b->cpu_count <= 0 || out_count <= 0)
    {
        return set_error("invalid cpu count in get_usage_between");
    }

    if (a->cpu_count != b->cpu_count)
    {
        /* Snapshots initialised with different cpu_counts (e.g. taken
         * across a CPU hotplug event) cannot be meaningfully compared. */
        return set_error("snapshot cpu_count mismatch");
    }

    if (out_count < a->cpu_count)
    {
        return set_error("output array too small");
    }

    for (int i = 0; i < a->cpu_count; ++i)
    {
        /* Step 1: compute the total jiffy delta (denominator). */
        unsigned long long total_diff = b->cores[i].total - a->cores[i].total;

        /* Step 2: compute the combined idle delta (idle + iowait).
         * iowait is included because from the application's perspective
         * a core waiting for I/O is still not doing useful compute work. */
        unsigned long long idle_a = a->cores[i].idle + a->cores[i].iowait;
        unsigned long long idle_b = b->cores[i].idle + b->cores[i].iowait;
        unsigned long long idle_diff = idle_b - idle_a;

        /* Step 3: apply usage formula, guard against division by zero,
         * and clamp to [0, 100]. */
        double usage = 0.0;
        if (total_diff > 0)
        {
            usage = 100.0 * (1.0 - ((double)idle_diff / (double)total_diff));
            if (usage < 0.0) usage = 0.0;
            if (usage > 100.0) usage = 100.0;
        }

        out_array[i].core_id       = i;
        out_array[i].usage_percent = usage;
        out_array[i].total_diff    = total_diff;
        out_array[i].idle_diff     = idle_diff;
    }

    return a->cpu_count;
}

int get_core_usage_percent(int core_id,
                                unsigned int interval_ms,
                                double *usage_percent)
{
    int cpu_count = get_online_cpu_count();
    if (cpu_count < 0)
    {
        return -1;
    }

    if (core_id < 0 || core_id >= cpu_count || !usage_percent)
    {
        return set_error("invalid arguments to get_core_usage_percent");
    }

    snapshot_t a = {0}, b = {0};
    core_usage_t *usage = NULL;
    int ret = -1;

    /* do-while(0) acts as a structured goto: break jumps directly to the
     * cleanup block below, ensuring resources are always released. */
    do {
        if (snapshot_init(&a, cpu_count) != 0) break;
        if (snapshot_init(&b, cpu_count) != 0) break;

        /* Take the "before" snapshot, wait for the sampling window, then
         * take the "after" snapshot.  The delta between them captures CPU
         * activity during that window.  Default interval is 1 second. */
        if (snapshot_take(&a) != 0) break;
        usleep(interval_ms ? interval_ms * 1000 : 1000000);
        if (snapshot_take(&b) != 0) break;

        usage = (core_usage_t *)calloc((size_t)cpu_count, sizeof(core_usage_t));
        if (!usage)
        {
            ret = set_error_errno("calloc failed", errno);
            break;
        }

        ret = get_usage_between(&a, &b, usage, cpu_count);
        if (ret < 0) break;

        /* Extract the result for the single requested core. */
        *usage_percent = usage[core_id].usage_percent;
        ret = 0;
    } while (0);

    /* Always release resources, even on error. */
    free(usage);
    snapshot_free(&a);
    snapshot_free(&b);
    return ret;
}

int get_all_core_usage(unsigned int interval_ms,
                            core_usage_t *out_array,
                            int out_count)
{
    int cpu_count = get_online_cpu_count();
    if (cpu_count < 0)
    {
        return -1;
    }

    snapshot_t a = {0}, b = {0};
    int ret = -1;

    if (!out_array || out_count < cpu_count)
    {
        return set_error("output array is null or too small");
    }

    /* Same snapshot-sleep-snapshot pattern as get_core_usage_percent(),
     * but results are written directly into the caller-supplied array
     * (no intermediate heap allocation needed). */
    do {
        if (snapshot_init(&a, cpu_count) != 0) break;
        if (snapshot_init(&b, cpu_count) != 0) break;

        if (snapshot_take(&a) != 0) break;
        usleep(interval_ms ? interval_ms * 1000 : 1000000);
        if (snapshot_take(&b) != 0) break;

        ret = get_usage_between(&a, &b, out_array, out_count);
    } while (0);

    snapshot_free(&a);
    snapshot_free(&b);
    return ret;
}

int find_least_used_core(unsigned int interval_ms,
                              int *core_id,
                              double *usage_percent)
{
    int cpu_count = get_online_cpu_count();
    if (cpu_count < 0)
    {
        return -1;
    }

    core_usage_t *usage = (core_usage_t *)calloc((size_t)cpu_count, sizeof(core_usage_t));
    if (!usage)
    {
        return set_error_errno("calloc failed", errno);
    }

    int ret = get_all_core_usage(interval_ms, usage, cpu_count);
    if (ret < 0)
    {
        free(usage);
        return -1;
    }

    /* Linear scan: seed with core 0, update when a lower utilisation is
     * found.  Ties are broken in favour of the lower-indexed core. */
    int best_core = 0;
    double best_usage = usage[0].usage_percent;

    for (int i = 1; i < cpu_count; ++i)
    {
        if (usage[i].usage_percent < best_usage)
        {
            best_usage = usage[i].usage_percent;
            best_core = i;
        }
    }

    /* Both output parameters are optional; callers may pass NULL. */
    if (core_id) *core_id = best_core;
    if (usage_percent) *usage_percent = best_usage;

    free(usage);
    return 0;
}

int find_most_used_core(unsigned int interval_ms,
                             int *core_id,
                             double *usage_percent)
{
    int cpu_count = get_online_cpu_count();
    if (cpu_count < 0)
    {
        return -1;
    }

    core_usage_t *usage = (core_usage_t *)calloc((size_t)cpu_count, sizeof(core_usage_t));
    if (!usage)
    {
        return set_error_errno("calloc failed", errno);
    }

    int ret = get_all_core_usage(interval_ms, usage, cpu_count);
    if (ret < 0)
    {
        free(usage);
        return -1;
    }

    /* Linear scan: seed with core 0, update when a higher utilisation is
     * found.  Ties are broken in favour of the lower-indexed core. */
    int best_core = 0;
    double best_usage = usage[0].usage_percent;

    for (int i = 1; i < cpu_count; ++i)
    {
        if (usage[i].usage_percent > best_usage)
        {
            best_usage = usage[i].usage_percent;
            best_core = i;
        }
    }

    /* Both output parameters are optional; callers may pass NULL. */
    if (core_id) *core_id = best_core;
    if (usage_percent) *usage_percent = best_usage;

    free(usage);
    return 0;
}

/* -------------------------------------------------------------------------
 * Public: cpu_set_t helpers
 * ---------------------------------------------------------------------- */

void cpuset_zero(cpu_set_t *set)
{
    if (set)
    {
        CPU_ZERO(set); /* clears all CPU bits in the set */
    }
}

void cpuset_set(cpu_set_t *set, int cpu_id)
{
    if (set && cpu_id >= 0)
    {
        CPU_SET(cpu_id, set); /* enables the bit for cpu_id */
    }
}

int cpuset_isset(const cpu_set_t *set, int cpu_id)
{
    if (!set || cpu_id < 0)
    {
        return 0;
    }
    return CPU_ISSET(cpu_id, set) ? 1 : 0;
}

int cpuset_count(const cpu_set_t *set)
{
    if (!set)
    {
        return 0;
    }
    return CPU_COUNT(set); /* returns the number of set bits */
}

int cpuset_to_string(const cpu_set_t *set,
                          char *buffer,
                          size_t buffer_size)
{
    if (!set || !buffer || buffer_size == 0)
    {
        return set_error("invalid arguments to cpuset_to_string");
    }

    /* Use the configured (not just online) count so that offline cores
     * still present in the mask are also included in the output string. */
    int cpu_count = get_configured_cpu_count();
    if (cpu_count < 0)
    {
        return -1;
    }

    size_t used = 0;
    buffer[0] = '\0'; /* guarantee NUL-termination even for an empty set */

    for (int i = 0; i < cpu_count; ++i)
    {
        if (!CPU_ISSET(i, set))
        {
            continue;
        }

        /* Prepend a comma separator for every element after the first. */
        int written = snprintf(buffer + used, buffer_size - used,
                               used ? ",%d" : "%d", i);
        if (written < 0 || (size_t)written >= buffer_size - used)
        {
            return set_error("buffer too small in cpuset_to_string");
        }
        used += (size_t)written;
    }

    return 0;
}

/* -------------------------------------------------------------------------
 * Public: thread affinity
 * ---------------------------------------------------------------------- */

int thread_set_affinity(pthread_t thread, const cpu_set_t *set)
{
    if (!set)
    {
        return set_error("null cpuset in thread_set_affinity");
    }

    /* pthread_setaffinity_np restricts the thread to the set of cores
     * described by *set.  The OS will migrate the thread on the next
     * scheduling event if it is currently running on an excluded core. */
    int ret = pthread_setaffinity_np(thread, sizeof(cpu_set_t), set);
    if (ret != 0)
    {
        return set_error_errno("pthread_setaffinity_np failed", ret);
    }
    return 0;
}

int thread_get_affinity(pthread_t thread, cpu_set_t *set)
{
    if (!set)
    {
        return set_error("null cpuset in thread_get_affinity");
    }

    /* Zero the set first so that bits for unavailable cores are always 0,
     * regardless of what the kernel writes into the opaque cpu_set_t. */
    CPU_ZERO(set);
    int ret = pthread_getaffinity_np(thread, sizeof(cpu_set_t), set);
    if (ret != 0)
    {
        return set_error_errno("pthread_getaffinity_np failed", ret);
    }
    return 0;
}

int thread_bind_to_cpu(pthread_t thread, int cpu_id)
{
    int cpu_count = get_online_cpu_count();
    if (cpu_count < 0)
    {
        return -1;
    }

    if (cpu_id < 0 || cpu_id >= cpu_count)
    {
        return set_error("invalid cpu_id in thread_bind_to_cpu");
    }

    /* Build a single-bit affinity mask and apply it. */
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu_id, &set);

    return thread_set_affinity(thread, &set);
}

int thread_bind_self_to_cpu(int cpu_id)
{
    /* pthread_self() always succeeds; no need to check for errors. */
    return thread_bind_to_cpu(pthread_self(), cpu_id);
}

int thread_bind_self_to_least_used_cpu(unsigned int interval_ms,
                                            int *selected_cpu,
                                            double *usage_percent)
{
    int cpu_id = -1;
    double usage = 0.0;

    /* Step 1: measure all cores and pick the one with the lowest load.
     * This call blocks for interval_ms milliseconds (default 1 s). */
    if (find_least_used_core(interval_ms, &cpu_id, &usage) != 0)
    {
        return -1;
    }

    /* Step 2: restrict the calling thread's affinity to that single core.
     * The OS will migrate the thread at the next scheduling point if it is
     * currently running on a different core. */
    if (thread_bind_self_to_cpu(cpu_id) != 0)
    {
        return -1;
    }

    /* Report the result to the caller (both parameters are optional). */
    if (selected_cpu) *selected_cpu = cpu_id;
    if (usage_percent) *usage_percent = usage;
    return 0;
}

/* -------------------------------------------------------------------------
 * Public: process affinity
 * ---------------------------------------------------------------------- */

int process_set_affinity(pid_t pid, const cpu_set_t *set)
{
    if (!set)
    {
        return set_error("null cpuset in process_set_affinity");
    }

    /* sched_setaffinity(2) affects the entire process (all its threads).
     * Pass pid=0 to target the calling process. */
    if (sched_setaffinity(pid, sizeof(cpu_set_t), set) != 0)
    {
        return set_error_errno("sched_setaffinity failed", errno);
    }
    return 0;
}

int process_get_affinity(pid_t pid, cpu_set_t *set)
{
    if (!set)
    {
        return set_error("null cpuset in process_get_affinity");
    }

    CPU_ZERO(set);
    if (sched_getaffinity(pid, sizeof(cpu_set_t), set) != 0)
    {
        return set_error_errno("sched_getaffinity failed", errno);
    }
    return 0;
}

int process_bind_to_cpu(pid_t pid, int cpu_id)
{
    int cpu_count = get_online_cpu_count();
    if (cpu_count < 0)
    {
        return -1;
    }

    if (cpu_id < 0 || cpu_id >= cpu_count)
    {
        return set_error("invalid cpu_id in process_bind_to_cpu");
    }

    /* Build a single-bit affinity mask and apply it to the whole process. */
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu_id, &set);

    return process_set_affinity(pid, &set);
}

/* -------------------------------------------------------------------------
 * Public: thread scheduling policy
 * ---------------------------------------------------------------------- */

int thread_set_sched_policy(pthread_t thread, int policy, int priority)
{
    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = priority;

    /* pthread_setschedparam(3) requires CAP_SYS_NICE or root privileges
     * when elevating to a real-time policy (SCHED_FIFO / SCHED_RR).
     * SCHED_OTHER / SCHED_BATCH / SCHED_IDLE must use priority 0. */
    int ret = pthread_setschedparam(thread, policy, &param);
    if (ret != 0)
    {
        return set_error_errno("pthread_setschedparam failed", ret);
    }
    return 0;
}

int thread_set_self_sched_policy(int policy, int priority)
{
    return thread_set_sched_policy(pthread_self(), policy, priority);
}

int thread_get_sched_policy(pthread_t thread, int *policy, int *priority)
{
    if (!policy || !priority)
    {
        return set_error("invalid arguments to thread_get_sched_policy");
    }

    struct sched_param param;
    memset(&param, 0, sizeof(param));

    int ret = pthread_getschedparam(thread, policy, &param);
    if (ret != 0)
    {
        return set_error_errno("pthread_getschedparam failed", ret);
    }

    *priority = param.sched_priority;
    return 0;
}

int set_current_thread_nice(int nice_value)
{
    /* POSIX getpid() returns the process ID.  On Linux, a per-thread nice
     * value must be set using the kernel thread ID (TID) obtained via the
     * gettid syscall.  glibc only exposed gettid() as a function from
     * version 2.30; using SYS_gettid directly ensures compatibility with
     * older toolchains. */
    pid_t tid = (pid_t)syscall(SYS_gettid);
    if (setpriority(PRIO_PROCESS, (id_t)tid, nice_value) != 0)
    {
        return set_error_errno("setpriority failed", errno);
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * Public: process scheduling policy
 * ---------------------------------------------------------------------- */

int process_set_sched_policy(pid_t pid, int policy, int priority)
{
    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = priority;

    /* sched_setscheduler(2) changes the policy for the entire process and
     * requires CAP_SYS_NICE for real-time policies.  Pass pid=0 to target
     * the calling process. */
    if (sched_setscheduler(pid, policy, &param) != 0)
    {
        return set_error_errno("sched_setscheduler failed", errno);
    }
    return 0;
}

int process_set_self_sched_policy(int policy, int priority)
{
    return process_set_sched_policy(getpid(), policy, priority);
}

int process_get_sched_policy(pid_t pid, int *policy, int *priority)
{
    if (!policy || !priority)
    {
        return set_error("invalid arguments to process_get_sched_policy");
    }

    struct sched_param param;
    memset(&param, 0, sizeof(param));

    /* sched_getscheduler(2) returns the policy on success or -1 on error,
     * so we cannot use the generic "ret != 0" error check here. */
    int ret = sched_getscheduler(pid);
    if (ret < 0)
    {
        return set_error_errno("sched_getscheduler failed", errno);
    }
    *policy = ret;

    /* sched_getparam(2) fills in the priority for the current policy. */
    if (sched_getparam(pid, &param) != 0)
    {
        return set_error_errno("sched_getparam failed", errno);
    }
    *priority = param.sched_priority;
    return 0;
}

int set_current_process_nice(int nice_value)
{
    /* setpriority(PRIO_PROCESS, pid, ...) sets the nice value for all
     * threads of the process that have not overridden it individually.
     * Valid range is typically [-20, 19]; higher means lower priority.
     * Lowering below the current value requires CAP_SYS_NICE. */
    if (setpriority(PRIO_PROCESS, getpid(), nice_value) != 0)
    {
        return set_error_errno("setpriority failed", errno);
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * Public: informational helpers
 * ---------------------------------------------------------------------- */

const char *policy_to_string(int policy)
{
    /* SCHED_BATCH and SCHED_IDLE are Linux-specific extensions, guarded with
     * #ifdef so the code still compiles on non-Linux platforms. */
    switch (policy)
    {
        case SCHED_OTHER: return "SCHED_OTHER"; /* default CFS time-sharing */
#ifdef SCHED_BATCH
        case SCHED_BATCH: return "SCHED_BATCH"; /* CPU-bound batch, no preemption bonus */
#endif
#ifdef SCHED_IDLE
        case SCHED_IDLE:  return "SCHED_IDLE";  /* very-low-priority background tasks */
#endif
        case SCHED_FIFO:  return "SCHED_FIFO";  /* real-time FIFO (no time slices) */
        case SCHED_RR:    return "SCHED_RR";    /* real-time round-robin */
        default:          return "UNKNOWN";
    }
}

} // namespace cpu_utils
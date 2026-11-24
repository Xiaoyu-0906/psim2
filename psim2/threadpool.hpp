// Persistent worker pool with a blocking parallel_for over [0, n).
// Work items are claimed dynamically in chunks so heterogeneous cores
// (performance + efficiency) stay busy. Callers must only write data owned
// by the item index; results are therefore independent of the thread count.
//
// Synchronization: a parallel region publishes its job under a new region number; chunks are claimed from a
// counter that carries the region number (a worker that wakes up late cannot claim items of a later region),
// and the region ends when all items are done. The calling thread does not wait for workers that did not take
// part (on a busy machine a descheduled idle worker would otherwise stall every region). Waiting threads spin
// for a short while before they block on a condition variable, so the many short regions of a step
// (lane-changing stages) do not pay a kernel wake-up each.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#if defined(__APPLE__)
#include <pthread.h>
#include <sys/qos.h>
#endif

namespace ps {

class ThreadPool {
public:
    explicit ThreadPool(int nThreads) : myN(nThreads < 1 ? 1 : nThreads) {
        for (int i = 1; i < myN; ++i) {
            myWorkers.emplace_back([this] { workerLoop(); });
        }
        setQoS();
    }

    ~ThreadPool() {
        myStop.store(true);
        {
            std::lock_guard<std::mutex> lk(myMutex);
            myGeneration.fetch_add(1);
        }
        myCv.notify_all();
        for (auto& t : myWorkers) {
            t.join();
        }
    }

    int size() const { return myN; }

    // Statistics per phase tag: number of parallel regions, their wall time and the time the threads
    // spent working in them (wall * threads - busy = waiting: load imbalance and synchronization).
    enum Tag { T_PLAN, T_APPROACH, T_EXEC, T_INTEGRATE, T_PARTIALS, T_LC_SNAP, T_LC, T_LC_PUBLISH, T_PED, T_COLLISION, T_OTHER, T_NUM };
    struct Stat {
        long long calls = 0, serialCalls = 0;
        double wall = 0, busy = 0;
    };
    void setTag(Tag t) { myTag = t; }
    const Stat& stat(int t) const { return myStats[t]; }
    static const char* tagName(int t) {
        static const char* names[] = {"plan", "approach", "exec", "integrate", "partials", "lcSnap", "lc", "lcPublish", "ped", "collision", "other"};
        return names[t];
    }

    // Runs f(i) for i in [0, n). grain = items claimed per fetch.
    void parallel_for(int n, const std::function<void(int)>& f, int grain = 16) {
        if (n <= 0) {
            return;
        }
        Stat& st = myStats[myTag];
        const auto t0 = std::chrono::steady_clock::now();
        if (myN == 1 || n <= grain) {
            for (int i = 0; i < n; ++i) {
                f(i);
            }
            const double d = seconds(t0);
            st.serialCalls++;
            st.wall += d;
            st.busy += d * myN;  // the other threads are not involved
            return;
        }
        // (the previous region was closed: its counter can no longer be claimed with the new item count)
        const uint64_t region = ++myRegion;
        myJob.store(&f, std::memory_order_relaxed);
        myJobN.store(n, std::memory_order_relaxed);
        myGrain.store(grain, std::memory_order_relaxed);
        myDone.store(0, std::memory_order_relaxed);
        const long long busy0 = myWorkerBusyNs.load(std::memory_order_relaxed);
        myClaim.store(region << 32, std::memory_order_release);
        myGeneration.fetch_add(1);  // seq_cst: pairs with the sleeper count of the workers
        if (mySleepers.load() > 0) {
            std::lock_guard<std::mutex> lk(myMutex);
            myCv.notify_all();
        }
        const auto m0 = std::chrono::steady_clock::now();
        runChunks();
        double busy = seconds(m0);
        // wait for the items claimed by the workers: spin, yield, then block
        bool done = false;
        for (int i = 0; i < SPIN + YIELDS && !done; ++i) {
            done = myDone.load(std::memory_order_acquire) == n;
            if (!done) {
                if (i < SPIN) {
                    relax();
                } else {
                    std::this_thread::yield();
                }
            }
        }
        if (!done) {
            std::unique_lock<std::mutex> lk(myMutex);
            myMainWaiting.store(true);
            myDoneCv.wait(lk, [&] { return myDone.load() == n; });
            myMainWaiting.store(false);
        }
        myClaim.store((region << 32) | CLOSED, std::memory_order_release);
        // (workers still finishing their timing of this region are counted in the next one)
        busy += 1e-9 * (double)(myWorkerBusyNs.load(std::memory_order_relaxed) - busy0);
        st.calls++;
        st.wall += seconds(t0);
        st.busy += busy;
    }

private:
    static constexpr int SPIN = 4000;   // pause instructions (a few microseconds)
    static constexpr int YIELDS = 32;
    static constexpr uint64_t CLOSED = 0x7fffffffULL;  // claim counter of a finished region

    static double seconds(const std::chrono::steady_clock::time_point& t0) {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    }

    static inline void relax() {
#if defined(__aarch64__)
        asm volatile("yield");
#elif defined(__x86_64__)
        __builtin_ia32_pause();
#endif
    }

    static void setQoS() {
#if defined(__APPLE__)
        // prefer performance cores for simulation threads
        pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
    }

    // claims and runs chunks of the region that is current at the first claim; returns whether it ran any
    bool runChunks() {
        uint64_t c = myClaim.load(std::memory_order_acquire);
        const uint64_t region = c >> 32;
        bool ran = false;
        while (true) {
            if ((c >> 32) != region) {
                break;  // a later region
            }
            const int start = (int)(c & 0xffffffffULL);
            const int n = myJobN.load(std::memory_order_relaxed);
            const int g = myGrain.load(std::memory_order_relaxed);
            if (start >= n) {
                break;
            }
            const int end = std::min(n, start + g);
            if (!myClaim.compare_exchange_weak(c, c + (uint64_t)(end - start), std::memory_order_acq_rel, std::memory_order_acquire)) {
                continue;  // (c was reloaded)
            }
            // the region cannot end before these items are done: the job is still the claimed one
            const std::function<void(int)>& f = *myJob.load(std::memory_order_relaxed);
            for (int i = start; i < end; ++i) {
                f(i);
            }
            ran = true;
            if (myDone.fetch_add(end - start, std::memory_order_acq_rel) + (end - start) == n && myMainWaiting.load()) {
                std::lock_guard<std::mutex> lk(myMutex);
                myDoneCv.notify_one();
            }
            c = myClaim.load(std::memory_order_acquire);
        }
        return ran;
    }

    void workerLoop() {
        setQoS();
        uint64_t seen = 0;
        while (true) {
            // wait for the next region: spin, then block
            uint64_t g = myGeneration.load(std::memory_order_acquire);
            for (int i = 0; i < SPIN + YIELDS && g == seen; ++i) {
                if (i < SPIN) {
                    relax();
                } else {
                    std::this_thread::yield();  // an overloaded machine: let the descheduled threads run
                }
                g = myGeneration.load(std::memory_order_acquire);
            }
            if (g == seen) {
                std::unique_lock<std::mutex> lk(myMutex);
                mySleepers.fetch_add(1);
                myCv.wait(lk, [&] { return myGeneration.load() != seen; });
                mySleepers.fetch_sub(1);
                g = myGeneration.load();
            }
            seen = g;
            if (myStop.load()) {
                return;
            }
            const auto w0 = std::chrono::steady_clock::now();
            if (runChunks()) {
                myWorkerBusyNs.fetch_add(
                    (long long)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - w0).count(),
                    std::memory_order_relaxed);
            }
        }
    }

    int myN;
    std::vector<std::thread> myWorkers;
    std::mutex myMutex;
    std::condition_variable myCv;
    std::condition_variable myDoneCv;
    std::atomic<const std::function<void(int)>*> myJob{nullptr};
    std::atomic<int> myJobN{0};
    std::atomic<int> myGrain{16};
    uint64_t myRegion = 0;                // (main thread only)
    std::atomic<uint64_t> myClaim{0};     // region << 32 | next item
    std::atomic<int> myDone{0};           // items done in the current region
    std::atomic<uint64_t> myGeneration{0};
    std::atomic<int> mySleepers{0};
    std::atomic<bool> myMainWaiting{false};
    std::atomic<bool> myStop{false};
    std::atomic<long long> myWorkerBusyNs{0};
    Tag myTag = T_OTHER;
    Stat myStats[T_NUM];
};

}  // namespace ps

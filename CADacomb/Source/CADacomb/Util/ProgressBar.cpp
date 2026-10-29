#include "ProgressBar.h"

namespace CADacomb
{
    thread_local std::string               ProgressBar::threadGuid;
    thread_local std::vector<ProgressBar*> ProgressBar::stack;
    thread_local std::atomic<bool>         ProgressBar::threadCanceled{ false };

    std::mutex                                                  ProgressBar::registryMutex;
    std::unordered_map<std::string, std::vector<ProgressBar*>*> ProgressBar::activeStacks;
    std::unordered_map<std::string, std::atomic<bool>*>         ProgressBar::activeCancelFlags;

    ProgressBar::ProgressBar(size_t maxProgress, const std::string& message)
      : maxProgress(maxProgress)
      , message(message)
      , cancelToken(&threadCanceled)
    {
        currentProgress.store(0, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(registryMutex);
        stack.push_back(this);
        if (!threadGuid.empty())
        {
            activeStacks[threadGuid]      = &stack;
            activeCancelFlags[threadGuid] = &threadCanceled;
        }
    }

    ProgressBar::~ProgressBar()
    {
        std::lock_guard<std::mutex> lock(registryMutex);
        if (!stack.empty() && stack.back() == this)
        {
            stack.pop_back();
        }
    }

    void ProgressBar::addProgress(size_t amount)
    {
        currentProgress.fetch_add(amount, std::memory_order_relaxed);
    }

    size_t ProgressBar::getCurrentProgress() const
    {
        return currentProgress.load(std::memory_order_relaxed);
    }

    size_t ProgressBar::getMaxProgress() const
    {
        return maxProgress;
    }

    const std::string& ProgressBar::getMessage() const
    {
        return message;
    }

    bool ProgressBar::isCanceled() const
    {
        if (cancelToken)
        {
            return cancelToken->load(std::memory_order_relaxed);
        }
        return threadCanceled.load(std::memory_order_relaxed);
    }

    void ProgressBar::setThreadGuid(const GUID& guid)
    {
        std::lock_guard<std::mutex> lock(registryMutex);
        if (!threadGuid.empty())
        {
            activeStacks.erase(threadGuid);
            activeCancelFlags.erase(threadGuid);
        }
        threadGuid = guid.toString();
        threadCanceled.store(false, std::memory_order_relaxed);
        if (!threadGuid.empty())
        {
            activeStacks[threadGuid]      = &stack;
            activeCancelFlags[threadGuid] = &threadCanceled;
        }
    }

    std::vector<ProgressBarStatus> ProgressBar::getProgress(const GUID& guid)
    {
        std::lock_guard<std::mutex>    lock(registryMutex);
        std::vector<ProgressBarStatus> result;

        auto it = activeStacks.find(guid.toString());
        if (it != activeStacks.end() && it->second)
        {
            result.reserve(it->second->size());
            for (ProgressBar* bar : *(it->second))
            {
                if (bar)
                {
                    result.push_back({ bar->getCurrentProgress(), bar->getMaxProgress(), bar->getMessage() });
                }
            }
        }

        return result;
    }

    void ProgressBar::cancel(const GUID& guid)
    {
        std::lock_guard<std::mutex> lock(registryMutex);
        auto                        it = activeCancelFlags.find(guid.toString());
        if (it != activeCancelFlags.end() && it->second)
        {
            it->second->store(true, std::memory_order_release);
        }
    }

    bool ProgressBar::isCanceled(const GUID& guid)
    {
        std::lock_guard<std::mutex> lock(registryMutex);
        auto                        it = activeCancelFlags.find(guid.toString());
        if (it != activeCancelFlags.end() && it->second)
        {
            return it->second->load(std::memory_order_relaxed);
        }
        return false;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <thread>

TEST_CASE("ProgressBar/BasicIncrement")
{
    CADacomb::ProgressBar bar(100, "Step 1");
    CHECK(bar.getMaxProgress() == 100);
    CHECK(bar.getCurrentProgress() == 0);
    CHECK(bar.getMessage() == "Step 1");

    bar.addProgress(10);
    CHECK(bar.getCurrentProgress() == 10);

    bar.addProgress();
    CHECK(bar.getCurrentProgress() == 11);
}

TEST_CASE("ProgressBar/ThreadSafeAddProgress")
{
    CADacomb::ProgressBar bar(1000, "Parallel Work");

    const int   iterationsPerThread = 250;
    std::thread t1(
      [&bar]()
      {
          for (int i = 0; i < iterationsPerThread; i++)
          {
              bar.addProgress(1);
          }
      });
    std::thread t2(
      [&bar]()
      {
          for (int i = 0; i < iterationsPerThread; i++)
          {
              bar.addProgress(1);
          }
      });
    std::thread t3(
      [&bar]()
      {
          for (int i = 0; i < iterationsPerThread; i++)
          {
              bar.addProgress(1);
          }
      });
    std::thread t4(
      [&bar]()
      {
          for (int i = 0; i < iterationsPerThread; i++)
          {
              bar.addProgress(1);
          }
      });

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    CHECK(bar.getCurrentProgress() == 1000);
}

TEST_CASE("ProgressBar/GuidQueryAndNesting")
{
    CADacomb::GUID guid = CADacomb::GUID::random();

    std::atomic<bool> workerReady{ false };
    std::atomic<bool> allowChildExit{ false };

    std::thread worker(
      [guid, &workerReady, &allowChildExit]()
      {
          CADacomb::ProgressBar::setThreadGuid(guid);
          CADacomb::ProgressBar parent(7, "Overall Pipeline");
          parent.addProgress(1);

          {
              CADacomb::ProgressBar child(100, "Landscape Generator");
              child.addProgress(45);

              workerReady.store(true);
              while (!allowChildExit.load())
              {
                  std::this_thread::sleep_for(std::chrono::milliseconds(1));
              }
          }
      });

    while (!workerReady.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::vector<CADacomb::ProgressBarStatus> statuses = CADacomb::ProgressBar::getProgress(guid);
    REQUIRE(statuses.size() == 2);
    CHECK(statuses[0].current == 1);
    CHECK(statuses[0].max == 7);
    CHECK(statuses[0].message == "Overall Pipeline");
    CHECK(statuses[1].current == 45);
    CHECK(statuses[1].max == 100);
    CHECK(statuses[1].message == "Landscape Generator");

    allowChildExit.store(true);
    worker.join();
}

TEST_CASE("ProgressBar/GuidCancellation")
{
    CADacomb::GUID guid = CADacomb::GUID::random();

    std::atomic<bool> workerStarted{ false };
    std::atomic<bool> workerSawCancel{ false };

    std::thread worker(
      [guid, &workerStarted, &workerSawCancel]()
      {
          CADacomb::ProgressBar::setThreadGuid(guid);
          CADacomb::ProgressBar bar(100, "Cancellable Job");

          workerStarted.store(true);
          while (!bar.isCanceled())
          {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          workerSawCancel.store(bar.isCanceled());
      });

    while (!workerStarted.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    CADacomb::ProgressBar::cancel(guid);
    worker.join();

    CHECK(workerSawCancel.load() == true);
}

TEST_CASE("ProgressBar/IndependentMultipleThreads")
{
    CADacomb::GUID guidA = CADacomb::GUID::random();
    CADacomb::GUID guidB = CADacomb::GUID::random();

    std::atomic<bool> bothReady{ false };
    std::atomic<bool> readyA{ false };
    std::atomic<bool> readyB{ false };

    std::thread threadA(
      [guidA, &readyA, &bothReady]()
      {
          CADacomb::ProgressBar::setThreadGuid(guidA);
          CADacomb::ProgressBar barA(50, "Task A");
          barA.addProgress(20);

          readyA.store(true);
          while (!bothReady.load())
          {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
      });

    std::thread threadB(
      [guidB, &readyB, &bothReady]()
      {
          CADacomb::ProgressBar::setThreadGuid(guidB);
          CADacomb::ProgressBar barB(200, "Task B");
          barB.addProgress(80);

          readyB.store(true);
          while (!bothReady.load())
          {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
      });

    while (!readyA.load() || !readyB.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    auto statusA = CADacomb::ProgressBar::getProgress(guidA);
    auto statusB = CADacomb::ProgressBar::getProgress(guidB);

    REQUIRE(statusA.size() == 1);
    CHECK(statusA[0].current == 20);
    CHECK(statusA[0].message == "Task A");

    REQUIRE(statusB.size() == 1);
    CHECK(statusB[0].current == 80);
    CHECK(statusB[0].message == "Task B");

    bothReady.store(true);
    threadA.join();
    threadB.join();
}
#endif

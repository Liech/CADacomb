#pragma once

#include "CADacomb/Util/GUID.h"

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace CADacomb
{
    struct ProgressBarStatus
    {
        size_t      current;
        size_t      max;
        std::string message;
    };

    // Thread-local RAII progress bar supporting nested steps, thread-safe increments, and GUID-based external querying and cancellation.
    class ProgressBar
    {
      public:
        ProgressBar(size_t maxProgress, const std::string& message);
        virtual ~ProgressBar();

        void               addProgress(size_t amount = 1);
        size_t             getCurrentProgress() const;
        size_t             getMaxProgress() const;
        const std::string& getMessage() const;

        bool isCanceled() const;

        static void                           setThreadGuid(const GUID& guid);
        static std::vector<ProgressBarStatus> getProgress(const GUID& guid);
        static void                           cancel(const GUID& guid);
        static bool                           isCanceled(const GUID& guid);

      private:
        std::atomic<size_t> currentProgress{ 0 };
        size_t              maxProgress = 0;
        std::string         message;

        static thread_local std::string               threadGuid;
        static thread_local std::vector<ProgressBar*> stack;
        static thread_local std::atomic<bool>         threadCanceled;

        static std::mutex                                                  registryMutex;
        static std::unordered_map<std::string, std::vector<ProgressBar*>*> activeStacks;
        static std::unordered_map<std::string, std::atomic<bool>*>         activeCancelFlags;

        std::atomic<bool>* cancelToken = nullptr;
    };
}

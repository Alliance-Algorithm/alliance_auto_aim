#pragma once

#include <tuple>

namespace world_exe::core {

template <typename... Args> struct SyncData {

    virtual std::tuple<Args&...> load() = 0;
    virtual void store(Args... args)    = 0;
    virtual ~SyncData()                 = default;
};
}
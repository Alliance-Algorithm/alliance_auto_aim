#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

namespace world_exe::interfaces {

template <class T> class ISyncLoad {

public:
    virtual void Store(const T& data) = 0;
    virtual std::optional<T> Load()   = 0;
};
}
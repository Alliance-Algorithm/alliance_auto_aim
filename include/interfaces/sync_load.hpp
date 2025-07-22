#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace world_exe::interfaces {

template <class T> class ISyncLoad {

public:
    virtual void Load(const T& data) = 0;
};
}
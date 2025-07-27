#pragma once

#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {
template <class T> class ISyncBlock {

public:
    virtual std::tuple<std::shared_ptr<T>, bool> await(double t_second) = 0;
};
}
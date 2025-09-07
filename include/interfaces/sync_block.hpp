#pragma once

#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {
/**
 * @brief 不必管他
 *
 * @tparam T
 */
template <class T> class ISyncBlock {

public:
    virtual std::tuple<std::shared_ptr<T>, bool> await(double t_second) = 0;
};
}
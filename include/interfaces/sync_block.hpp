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

    virtual void set_data(const T& camera_data);

    virtual std::tuple<T, bool> get_data(time_t timestamp);

    virtual ~ISyncBlock() = default;
};
}
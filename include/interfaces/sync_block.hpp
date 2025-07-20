#pragma once

#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {
template <class T> class ISyncBlock {

public:
    /// 如果没有在t_second时间内获取需要的所有数据，返回false
    virtual std::tuple<const T&, bool> await(double t_second = 2);
};
}
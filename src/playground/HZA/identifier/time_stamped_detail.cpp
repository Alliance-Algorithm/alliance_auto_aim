#include "time_stamped_detail.hpp"

namespace world_exe::interfaces::detail {
const std::time_t TimeStamped::GetTimeStamp() const {
    // 不是很清楚具体实现是什么
    return time_stamp_;
}
}
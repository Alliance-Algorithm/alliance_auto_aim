#pragma once

#include "data/fire_control.hpp"
#include "enum/car_id.hpp"
#include <ctime>

namespace world_exe::interfaces {

/**
 * @brief 火控系统，获取云台控制方向
 */
class IFireControl {
public:
    /**
     * @brief 计算云台和发射系统控制量
     *
     * @param time_duration 额外时间差 典型值：当前时刻到当前帧传感器传入内容的时间差
     * @return  data::FireControl
     */
    virtual data::FireControl CalculateTarget(data::TimeStamp const& time_stamp) const = 0;

    /**
     * @brief 获取当前火控系统锁定的车辆ID
     *
     * @return  enumeration::CarIDFlag : 火控系统锁定的车辆ID
     */
    virtual enumeration::CarIDFlag GetAttackCarId() const = 0;

    virtual ~IFireControl() = default;
};
}
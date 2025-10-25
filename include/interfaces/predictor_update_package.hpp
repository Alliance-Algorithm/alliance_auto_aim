#pragma once

#include "./armor_in_camera.hpp"

#include "interfaces/time_stamped.hpp"
#include <memory>

namespace world_exe::interfaces {

/**
 * @brief 给出滤波器需要的所有数据
 * @todo 应该使用结构体，但是先写着无所谓
 */
class IPreDictorUpdatePackage {
public:
    /**
     * @brief 传感器数据获取时的时间戳
     */
    virtual const ITimeStamped& GetTimeStamped() const = 0;
    /**
     * @brief 求解好的装甲板三维信息
     *
     * @return std::shared_ptr<IArmorInCamera>
     */
    virtual std::shared_ptr<IArmorInCamera> GetArmors() const = 0;

    /**
     * @brief 相机坐标系到世界坐标系的仿射变换
     *
     * @return Eigen::Affine3d
     */
    virtual Eigen::Affine3d GetTransform() const = 0;

    virtual ~IPreDictorUpdatePackage() = default;
};
}
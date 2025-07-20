#pragma once

#include "interfaces/armor_in_camera.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <type_traits>

namespace world_exe::interfaces {

template <typename T>
    requires std::is_same<T, IArmorInCamera>::value
class ISyncLoadArmor {

public:
    void Load(const T& data);
};
}
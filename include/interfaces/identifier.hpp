#pragma once

#include "interfaces/armor_in_image.hpp"
#include "interfaces/car_state.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {
class IIdentifier {
public:
    /// return : all armors , all armors id
    virtual const std::tuple<const std::shared_ptr<IArmorInImage>, enumeration::CarIDFlag> identify(
        const cv::Mat& input_image) = 0;
};
}
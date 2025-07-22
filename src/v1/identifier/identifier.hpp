#pragma once

#include "interfaces/identifier.hpp"

namespace world_exe::v1::identifier {
class Identifier : public interfaces::IIdentifier {
public:
    Identifier(const std::string& model_path, const std::string& device);
    ~Identifier();

    // false 为蓝色 ， true 为红色
    void SetTargetColor(bool target_color);

    const std::tuple<const interfaces::IArmorInImage&, enumeration::CarIDFlag> identify(
        const cv::Mat& input_image) override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
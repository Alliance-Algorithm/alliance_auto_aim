#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>

#include "enum/armor_id.hpp"
namespace world_exe::tongji::identifier {

class Classifier final {
public:
    Classifier(const std::string& model_path, int model_image_width, int model_image_height);
    ~Classifier();

    enumeration::ArmorIdFlag classify(const cv::Mat& armor_pattern);

    enumeration::ArmorIdFlag ovclassify(const cv::Mat& armor_pattern);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}
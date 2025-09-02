#include "classifier.hpp"
#include "enum/armor_id.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <openvino/runtime/core.hpp>

namespace world_exe::tongji::identifier {
class Classifier::Impl final {
public:
    explicit Impl(const std::string& model_path, int model_image_width, int model_image_height)
        : model_image_width_(model_image_width)
        , model_image_height_(model_image_height) {

        net_            = cv::dnn::readNetFromONNX(model_path);
        auto ovmodel    = core_.read_model(model_path);
        compiled_model_ = core_.compile_model(
            ovmodel, "AUTO", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    }

    enumeration::ArmorIdFlag classify(const cv::Mat& armor_pattern) {
        if (armor_pattern.empty()) {
            return enumeration::ArmorIdFlag::Unknow;
        }

        cv::Mat gray;
        cv::cvtColor(armor_pattern, gray, cv::COLOR_BGR2GRAY);

        auto input   = cv::Mat(model_image_height_, model_image_width_, CV_8UC1, cv::Scalar(0));
        auto x_scale = static_cast<double>(model_image_width_) / gray.cols;
        auto y_scale = static_cast<double>(model_image_height_) / gray.rows;
        auto scale   = std::min(x_scale, y_scale);
        auto h       = static_cast<int>(gray.rows * scale);
        auto w       = static_cast<int>(gray.cols * scale);

        if (h == 0 || w == 0) {
            return enumeration::ArmorIdFlag::Unknow;
        }
        auto roi = cv::Rect(0, 0, w, h);
        cv::resize(gray, input(roi), { w, h });

        auto blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());

        net_.setInput(blob);
        cv::Mat outputs = net_.forward();

        // softmax
        float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::exp(outputs - max, outputs);
        float sum = cv::sum(outputs)[0];
        outputs /= sum;

        double confidence;
        cv::Point label_point;
        cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
        int label_id = label_point.x;

        enumeration::ArmorIdFlag armor_id;

        if (label_id == 3) {
            armor_id = enumeration::ArmorIdFlag::InfantryIII;
        } else if (label_id == 4) {
            armor_id = enumeration::ArmorIdFlag::InfantryIV;
        } else if (label_id == 5) {
            armor_id = enumeration::ArmorIdFlag::InfantryV;
        } else if (label_id == 6) {
            armor_id = enumeration::ArmorIdFlag::Outpost;
        } else if (label_id == 1) {
            armor_id = enumeration::ArmorIdFlag::Hero;
        } else if (label_id == 2) {
            armor_id = enumeration::ArmorIdFlag::Engineer;
        } else if (label_id == 0) {
            armor_id = enumeration::ArmorIdFlag::Sentry;
        } else if (label_id == 7) {
            armor_id = enumeration::ArmorIdFlag::Base;
        };

        return armor_id;
    }

    enumeration::ArmorIdFlag ovclassify(const cv::Mat& armor_pattern) {
        if (armor_pattern.empty()) {
            return enumeration::ArmorIdFlag::Unknow;
        }

        cv::Mat gray;
        cv::cvtColor(armor_pattern, gray, cv::COLOR_BGR2GRAY);

        // Resize image
        auto input   = cv::Mat(model_image_height_, model_image_width_, CV_8UC1, cv::Scalar(0));
        auto x_scale = static_cast<double>(model_image_width_) / gray.cols;
        auto y_scale = static_cast<double>(model_image_height_) / gray.rows;
        auto scale   = std::min(x_scale, y_scale);
        auto h       = static_cast<int>(gray.rows * scale);
        auto w       = static_cast<int>(gray.cols * scale);

        if (h == 0 || w == 0) {
            return enumeration::ArmorIdFlag::Unknow;
        }

        auto roi = cv::Rect(0, 0, w, h);
        cv::resize(gray, input(roi), { w, h });
        // Normalize the input image to [0, 1] range
        input.convertTo(input, CV_32F, 1.0 / 255.0);

        ov::Tensor input_tensor(ov::element::f32, { 1, 1, 32, 32 }, input.data);

        ov::InferRequest infer_request = compiled_model_.create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();

        auto output_tensor = infer_request.get_output_tensor();
        auto output_shape  = output_tensor.get_shape();
        cv::Mat outputs(1, 9, CV_32F, output_tensor.data());

        // Softmax
        float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::exp(outputs - max, outputs);
        float sum = cv::sum(outputs)[0];
        outputs /= sum;

        double confidence;
        cv::Point label_point;
        cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
        int label_id = label_point.x;
        enumeration::ArmorIdFlag armor_id;

        if (label_id == 3) {
            armor_id = enumeration::ArmorIdFlag::InfantryIII;
        } else if (label_id == 4) {
            armor_id = enumeration::ArmorIdFlag::InfantryIV;
        } else if (label_id == 5) {
            armor_id = enumeration::ArmorIdFlag::InfantryV;
        } else if (label_id == 6) {
            armor_id = enumeration::ArmorIdFlag::Outpost;
        } else if (label_id == 1) {
            armor_id = enumeration::ArmorIdFlag::Hero;
        } else if (label_id == 2) {
            armor_id = enumeration::ArmorIdFlag::Engineer;
        } else if (label_id == 0) {
            armor_id = enumeration::ArmorIdFlag::Sentry;
        } else if (label_id == 7) {
            armor_id = enumeration::ArmorIdFlag::Base;
        };

        return armor_id;
    };

private:
    cv::dnn::Net net_;
    ov::Core core_;
    ov::CompiledModel compiled_model_;

    const int model_image_height_ = 640;
    const int model_image_width_  = 640;
};

Classifier::Classifier(const std::string& model_path, int model_image_width, int model_image_height)
    : pimpl_(std::make_unique<Impl>(model_path, model_image_width, model_image_height)) { }
Classifier::~Classifier() = default;

enumeration::ArmorIdFlag Classifier::classify(const cv::Mat& armor_pattern) {
    return pimpl_->classify(armor_pattern);
}
enumeration::ArmorIdFlag Classifier::ovclassify(const cv::Mat& armor_pattern){
    return pimpl_->ovclassify(armor_pattern);
}
}
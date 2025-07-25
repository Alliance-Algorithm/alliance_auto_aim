#include "identifier.hpp"
#include "identifier_armor.hpp"
#include "util/scanline.hpp"

#include "opencv2/dnn/dnn.hpp"
#include "opencv2/imgproc.hpp"

#include "openvino/core/preprocess/pre_post_process.hpp"
#include "openvino/runtime/core.hpp"
#include <memory>

namespace world_exe::v1::identifier {
class Identifier::Impl {
public:
    explicit Impl(const std::string& model_path, const std::string& device,
        const int& image_width = 1440, const int& image_height = 1080)
        : image_width_(image_width)
        , image_height_(image_height)
        , width_ratio_(static_cast<double>(image_width_) / model_image_width_)
        , height_ratio_(static_cast<double>(image_height_) / model_image_height_) {
        ov::Core core_;
        const auto adevice = core_.get_available_devices();
        auto model_        = core_.read_model(model_path);

        std::unique_ptr<ov::preprocess::PrePostProcessor> pre_post_processor_ =
            std::make_unique<ov::preprocess::PrePostProcessor>(model_);
        ov::Shape input_shape_ { 1, model_image_height_, model_image_width_, 3 };
        pre_post_processor_->input()
            .tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);

        pre_post_processor_->input()
            .preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale({ 255., 255., 255. });

        pre_post_processor_->input().model().set_layout("NCHW");
        pre_post_processor_->output().tensor().set_element_type(ov::element::f32);
        model_ = pre_post_processor_->build();

        compiled_model_ = core_.compile_model(model_, device);
    }

    inline void SetTargetColor(bool target_color) { target_color_ = target_color; }

    const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
    Identify(const cv::Mat& input_image) {
        const auto armor_infos = model_infer(input_image);
        return matchPlate(input_image, armor_infos);
    }

    inline void set_match_magnification_ratio(const double& ratio) {
        match_magnification_ratio_ = ratio;
    }

private:
    struct ArmorInfo {
        cv::Rect rect_;
        enumeration::ArmorIdFlag id_;
    };

    std::vector<ArmorInfo> model_infer(const cv::Mat& img) {
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(model_image_width_, model_image_height_));
        const auto input_tensor        = ov::Tensor { compiled_model_.input().get_element_type(),
            compiled_model_.input().get_shape(), resized_img.data };
        ov::InferRequest infer_request = compiled_model_.create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();

        const auto output        = infer_request.get_output_tensor(0);
        const auto& output_shape = output.get_shape();

        cv::Mat output_buffer(static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]),
            CV_32F, output.data());

        std::vector<cv::Rect> boxes;
        std::vector<int> class_ids;
        std::vector<float> class_scores;
        std::vector<float> confidences;
        std::vector<ArmorInfo> tmp_objects_;
        for (int i = 0; i < output_buffer.rows; i++) {
            float confidence = output_buffer.at<float>(i, 8);
            confidence       = static_cast<float>(sigmoid(confidence));
            if (confidence < conf_threshold_) continue;

            const auto color_scores   = output_buffer.row(i).colRange(9, 13);  // color
            const auto classes_scores = output_buffer.row(i).colRange(13, 22); // num
            cv::Point class_id, color_id;
            double score_color, score_num;
            cv::minMaxLoc(classes_scores, nullptr, &score_num, nullptr, &class_id);
            cv::minMaxLoc(color_scores, nullptr, &score_color, nullptr, &color_id);

            if (color_id.x >= 2 || (color_id.x == 1 && !target_color_)
                || (color_id.x == 0 && target_color_))
                continue;

            ArmorInfo obj;
            if (class_id.x == 3) {
                obj.id_ = enumeration::ArmorIdFlag::InfantryIII;
            } else if (class_id.x == 4) {
                obj.id_ = enumeration::ArmorIdFlag::InfantryIV;
            } else if (class_id.x == 5) {
                obj.id_ = enumeration::ArmorIdFlag::InfantryV;
            } else if (class_id.x == 6) {
                obj.id_ = enumeration::ArmorIdFlag::Outpost;
            } else if (class_id.x == 1) {
                obj.id_ = enumeration::ArmorIdFlag::Hero;
            } else if (class_id.x == 2) {
                obj.id_ = enumeration::ArmorIdFlag::Engineer;
            } else if (class_id.x == 0) {
                obj.id_ = enumeration::ArmorIdFlag::Sentry;
            } else if (class_id.x == 7) {
                obj.id_ = enumeration::ArmorIdFlag::Base;
            } else continue;

            tmp_objects_.emplace_back(obj);

            std::array<cv::Point2f, 4> points { cv::Point2f { output_buffer.at<float>(i, 0),
                                                    output_buffer.at<float>(i, 1) },
                cv::Point2f { output_buffer.at<float>(i, 6), output_buffer.at<float>(i, 7) },
                cv::Point2f { output_buffer.at<float>(i, 4), output_buffer.at<float>(i, 5) },
                cv::Point2f { output_buffer.at<float>(i, 2), output_buffer.at<float>(i, 3) } };
            float min_x = points[0].x;
            float max_x = points[0].x;
            float min_y = points[0].y;
            float max_y = points[0].y;
            for (std::size_t i = 1; i < points.size(); i++) {
                if (points[i].x < min_x) min_x = points[i].x;
                if (points[i].x > max_x) max_x = points[i].x;
                if (points[i].y < min_y) min_y = points[i].y;
                if (points[i].y > max_y) max_y = points[i].y;
            }

            boxes.emplace_back(min_x * width_ratio_, min_y * height_ratio_,
                (max_x - min_x) * width_ratio_, (max_y - min_y) * height_ratio_);
            confidences.emplace_back(score_num);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);
        std::vector<ArmorInfo> objects_;
        for (const std::size_t valid_index : indices)
            if (valid_index <= boxes.size()) {
                auto object  = tmp_objects_[valid_index];
                object.rect_ = boxes[valid_index];
                objects_.emplace_back(object);
            }

        return objects_;
    }

    static inline double sigmoid(double x) {
        if (x > 0) return 1.0 / (1.0 + std::exp(-x));
        else return std::exp(x) / (1.0 + std::exp(x));
    }

    struct LightBar {
        cv::Point2f top_, bottom_;
        float angle_;

        LightBar(const cv::Point2f& top, const cv::Point2f& bottom, float angle)
            : top_(top)
            , bottom_(bottom)
            , angle_(angle) { }
    };

    const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
    matchPlate(const cv::Mat& img, const std::vector<ArmorInfo>& armor_infos) {
        cv::Mat gray_img;
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_img, gray_img, 30, 255, cv::THRESH_BINARY);

        uint32_t all_car_id = static_cast<uint32_t>(enumeration::ArmorIdFlag::None);
        std::vector<data::ArmorImageSpacing> armor_plates;
        for (const auto& armor : armor_infos) {
            const auto offset = cv::Point {
                std::clamp(static_cast<int>(armor.rect_.x
                               - armor.rect_.width / 2. * (match_magnification_ratio_ - 1.)),
                    0, image_width_),
                std::clamp(static_cast<int>(armor.rect_.y
                               - armor.rect_.height / 2. * (match_magnification_ratio_ - 1.)),
                    0, image_width_)
            };

            cv::Size rect_size { std::clamp(static_cast<int>(
                                                armor.rect_.width * match_magnification_ratio_),
                                     0, image_width_ - offset.x),
                std::clamp(static_cast<int>(armor.rect_.height * match_magnification_ratio_), 0,
                    image_height_ - offset.y) };

            const auto armor_roi = gray_img(cv::Rect { offset, rect_size });

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(armor_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            if (contours.size() >= 2) {
                std::sort(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a, false) > cv::contourArea(b, false);
                    });

                double first_area { 0. };
                std::vector<LightBar> lightbars_;
                for (const auto& contour : contours) {
                    auto b_rect = cv::boundingRect(contour);

                    cv::Point roi_point { std::clamp(offset.x + b_rect.x, 0, image_width_),
                        std::clamp(offset.y + b_rect.y, 0, image_height_) };
                    cv::Size roi_size { std::clamp(b_rect.width, 0, image_width_ - roi_point.x),
                        std::clamp(b_rect.height, 0, image_height_ - roi_point.y) };
                    const auto light_bar_roi = img(cv::Rect { roi_point, roi_size });

                    const auto channels       = cv::mean(light_bar_roi);
                    const auto b_r_difference = channels[0] - channels[2];
                    if ((target_color_ && b_r_difference > 0)
                        || (!target_color_ && b_r_difference < 0))
                        continue;

                    const auto points = util::ScanLine::get_points(armor_roi, contour);
                    if (points.empty()) continue;
                    const auto [high_point, low_point] = perform_pca(points);

                    if (lightbars_.empty()) {
                        first_area = cv::contourArea(contour);
                        lightbars_.emplace_back(high_point + offset, low_point + offset, 0.);
                    } else {
                        const auto area_ratio = first_area / cv::contourArea(contour);
                        if (area_ratio < 10. && area_ratio > 1. / 10.)
                            lightbars_.emplace_back(high_point + offset, low_point + offset, 0.);
                    }

                    if (lightbars_.size() == 2) break;
                }

                const auto light_bar_size_ = lightbars_.size();

                if (light_bar_size_ == 2) {
                    const auto& first  = lightbars_[0];
                    const auto& second = lightbars_[1];
                    if ((first.top_.x + first.bottom_.x) / 2.
                        < (second.bottom_.x + second.top_.x) / 2.) {
                        armor_plates.emplace_back(data::ArmorImageSpacing { armor.id_,
                            { first.top_, second.top_, second.bottom_, first.bottom_ } });
                    } else {
                        armor_plates.emplace_back(data::ArmorImageSpacing { armor.id_,
                            { second.top_, first.top_, first.bottom_, second.bottom_ } });
                    }
                    all_car_id |= static_cast<uint32_t>(armor.id_);
                }
            }
        }

        return { std::make_shared<IdentifierArmor>(armor_plates),
            static_cast<enumeration::CarIDFlag>(all_car_id) };
    }

    static inline std::tuple<cv::Point, cv::Point> perform_pca(
        const std::vector<cv::Point>& points) {
        const int points_num { static_cast<int>(points.size()) };
        cv::Mat data(points_num, 2, CV_32F);
        for (int j = 0; j < points_num; ++j) {
            data.at<float>(j, 0) = static_cast<float>(points[j].x);
            data.at<float>(j, 1) = static_cast<float>(points[j].y);
        }

        cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
        cv::Vec2f principal_axis(
            pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
        cv::Point2f center(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));

        float min_proj = std::numeric_limits<float>::max();
        float max_proj = std::numeric_limits<float>::lowest();

        for (const auto& p : points) {
            float proj = (static_cast<cv::Point2f>(p) - center).dot(principal_axis);

            if (proj < min_proj) min_proj = proj;
            if (proj > max_proj) max_proj = proj;
        }

        const cv::Point reconstructed_min_point_local =
            center + cv::Point2f { principal_axis * min_proj };
        const cv::Point reconstructed_max_point_local =
            center + cv::Point2f { principal_axis * max_proj };

        if (reconstructed_min_point_local.y < reconstructed_max_point_local.y)
            return { reconstructed_min_point_local, reconstructed_max_point_local };
        else return { reconstructed_max_point_local, reconstructed_min_point_local };
    }

    static constexpr int model_image_height_ = 640;
    static constexpr int model_image_width_  = 640;
    int image_height_                        = 1080;
    int image_width_                         = 1440;
    double width_ratio_  = static_cast<double>(image_width_) / model_image_width_;
    double height_ratio_ = static_cast<double>(image_height_) / model_image_height_;
    static constexpr double conf_threshold_ = 0.65;
    static constexpr double nms_threshold_  = 0.45;

    double match_magnification_ratio_ = 1.5;

    bool target_color_ { false };
    ov::CompiledModel compiled_model_;
};

Identifier::Identifier(const std::string& model_path, const std::string& device,
    const int& image_width, const int& image_height)
    : pimpl_(std::make_unique<Impl>(model_path, device, image_width, image_height)) { }

void Identifier::SetTargetColor(bool target_color) { return pimpl_->SetTargetColor(target_color); }

const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
Identifier::identify(const cv::Mat& input_image) {
    return pimpl_->Identify(input_image);
};

void Identifier::set_match_magnification_ratio(const double& ratio) {
    return pimpl_->set_match_magnification_ratio(ratio);
}

Identifier::~Identifier() = default;
}
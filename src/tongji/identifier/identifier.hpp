#pragma once

#include "interfaces/identifier.hpp"

namespace world_exe::tongji::identifier {

enum Color { red, blue, extinguish, purple };

struct Lightbar {
    std::size_t id;
    Color color;
    cv::Point2f center, top, bottom, top2bottom;
    std::vector<cv::Point2f> points;
    double angle, angle_error, length, width, ratio;
    cv::RotatedRect rotated_rect;

    Lightbar(const cv::RotatedRect& rotated_rect, std::size_t id)
        : id(id)
        , rotated_rect(rotated_rect) {
        std::vector<cv::Point2f> corners(4);
        rotated_rect.points(&corners[0]);
        std::sort(corners.begin(), corners.end(),
            [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

        this->center     = rotated_rect.center;
        this->top        = (corners[0] + corners[1]) / 2.;
        this->bottom     = (corners[2] + corners[3]) / 2.;
        this->top2bottom = bottom - top;

        this->points.emplace_back(top);
        this->points.emplace_back(bottom);

        this->width       = cv::norm(corners[0] - corners[1]);
        this->angle       = std::atan2(top2bottom.y, top2bottom.x);
        this->angle_error = std::abs(angle - M_PI / 2);
        this->length      = cv::norm(top2bottom);
        this->ratio       = length / width;
    }
};

class IdentifierImpl;
class Identifier final : public interfaces::IIdentifier {
public:
    Identifier(const std::string& model_path, int model_image_width, int model_image_height);
    ~Identifier();

    const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
    identify(const cv::Mat& input_image) override;

    void SetTargetColor(Color target_color);

private:
    std::unique_ptr<IdentifierImpl> pimpl_;
};

}
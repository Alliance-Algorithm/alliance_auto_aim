#include <armor_identifier.hpp>

ArmorIdentifier::ArmorIdentifier(){}

void ArmorIdentifier::process_image(const cv::Mat& image)
{
    coordinates_.clear();

    image_raw = image.clone();
    cv::cvtColor(image_raw, image_hsv, cv::COLOR_BGR2HSV);

    // 蓝色掩膜
    cv::Scalar lower_blue(100, 43, 46);
    cv::Scalar upper_blue(124, 255, 255);
    cv::inRange(image_hsv, lower_blue, upper_blue, mask_blue);

    // 红色掩膜
    cv::Scalar lower_red1(0, 43, 46), upper_red1(10, 255, 255);
    cv::Scalar lower_red2(156, 43, 46), upper_red2(180, 255, 255);
    cv::inRange(image_hsv, lower_red1, upper_red1, mask_red1);
    cv::inRange(image_hsv, lower_red2, upper_red2, mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red);

    // 二值化
    cv::threshold(mask_red, mask_red, 128, 255, cv::THRESH_BINARY);
    cv::threshold(mask_blue, mask_blue, 128, 255, cv::THRESH_BINARY);

    // 形态学去噪
    cv::erode(mask_red, mask_red, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask_red, mask_red, cv::Mat(), cv::Point(-1, -1), 2);
    cv::erode(mask_blue, mask_blue, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask_blue, mask_blue, cv::Mat(), cv::Point(-1, -1), 2);

    // 获取轮廓
    std::vector<std::vector<cv::Point>> contours_red, contours_blue;
    cv::findContours(mask_red, contours_red, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(mask_blue, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 筛选合格灯条
    std::vector<cv::RotatedRect> valid_red_bars, valid_blue_bars;
    for(const auto& contour : contours_red) {
        cv::RotatedRect box = cv::minAreaRect(contour);
        float ratio = box.size.height / box.size.width;
        if(ratio > 4 && ratio < 15)
        {
            valid_red_bars.push_back(box);
            cv::Point2f pts[4];
            box.points(pts);
            for (int i = 0; i < 4; ++i) {
                cv::line(image_output, pts[i], pts[(i+1)%4], cv::Scalar(255,0,0), 2);
            }
        }
    }
    for(const auto& contour : contours_blue) {
        cv::RotatedRect box = cv::minAreaRect(contour);
        float ratio = box.size.height / box.size.width;
        if(ratio > 4 && ratio < 15)
        {
            valid_blue_bars.push_back(box);
            cv::Point2f pts[4];
            box.points(pts);
            for (int i = 0; i < 4; ++i) {
                cv::line(image_output, pts[i], pts[(i+1)%4], cv::Scalar(255,0,0), 2);
            }
        }
    }
    // 配对红色灯条
    pair_bars(valid_red_bars, "red");
    // 配对蓝色灯条
    pair_bars(valid_blue_bars, "blue");
}

void ArmorIdentifier::pair_bars(const std::vector<cv::RotatedRect>& bars,
                                const std::string & color)
{
    // 配对红色灯条
    for(size_t i = 0; i < bars.size(); i++) {
        for(size_t j = i+1; j < bars.size(); j++) {
            cv::RotatedRect box1 = bars[i];
            cv::RotatedRect box2 = bars[j];
            float angle_diff = std::abs(box1.angle - box2.angle);
            float height_diff = std::abs(box1.size.height - box2.size.height);
            float distance_height = cv::norm(box1.center - box2.center);
            float distance_width = (box1.size.height + box2.size.height) / 2.0f;
            float ratio = distance_height / distance_width;
            if(angle_diff < 10 || height_diff < 20 || ratio > 2 || ratio < 2.5) {
                cv::Point2f pts1[4], pts2[4];
                box1.points(pts1);
                box2.points(pts2);
                std::vector<cv::Point2f> armor_pts = {pts1[0], pts1[1], pts1[2], pts1[3],
                                                      pts2[0], pts2[1], pts2[2], pts2[3]};
                cv::RotatedRect armor_box = cv::minAreaRect(armor_pts);
                cv::Point2f armor_box_pts[4];
                armor_box.points(armor_box_pts);
                ArmorCoordinates armor_coordinate;
                armor_coordinate.leftup    = armor_box_pts[1];
                armor_coordinate.rightup   = armor_box_pts[2];
                armor_coordinate.leftdown  = armor_box_pts[3];
                armor_coordinate.rightdown = armor_box_pts[0];
                armor_coordinate.center    = armor_box.center;
                armor_coordinate.is_identified = true;
                armor_coordinate.color = color;
                coordinates_.push_back(armor_coordinate);
            }
        }
    }
}

void ArmorIdentifier::show()
{
    image_output = image_raw.clone();

    // 画所有装甲板
    for(const auto& coordinate : coordinates_) {
        // 四个角点
        cv::Point2f pts[4] = {coordinate.leftup, coordinate.rightup, coordinate.rightdown, coordinate.leftdown};
        cv::Scalar color = (coordinate.color == "red") ? cv::Scalar(0,0,255) : cv::Scalar(255,0,0);

        for (int i = 0; i < 4; ++i) {
            cv::line(image_output, pts[i], pts[(i+1)%4], color, 2);
        }
        cv::circle(image_output, coordinate.center, 3, color, -1);
    }
    cv::imshow("Armor Identifier", image_output);
}

ArmorCoordinates ArmorIdentifier::get_coordinates_closest() const
{
    if(coordinates_.empty()) {
        return ArmorCoordinates(); // 返回默认值，表示未识别到装甲板
    }
    // 找到最靠近图像中心的装甲板
    cv::Point2f image_center(image_raw.cols / 2.0f, image_raw.rows / 2.0f);
    auto closest_armor = std::min_element(coordinates_.begin(), coordinates_.end(),
        [&image_center](const ArmorCoordinates& a, const ArmorCoordinates& b) {
            return cv::norm(a.center - image_center) < cv::norm(b.center - image_center);
        });
    return *closest_armor;
}
std::vector<ArmorCoordinates> ArmorIdentifier::get_coordinates_all() const
{
    return coordinates_;
}
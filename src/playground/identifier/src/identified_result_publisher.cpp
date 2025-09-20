#include "identified_result_publisher.hpp"


IdentifiedResultPublisher::IdentifiedResultPublisher()
: Node("identified_result_publisher")
{
    publisher_ = this->create_publisher<identifier::msg::PointData>("identifier_box", 10);
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("identified_image", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_markers", 10);
}

void IdentifiedResultPublisher::update_frame(const cv::Mat& frame)
{
    armor_identifier_.process_image(frame);
}

void IdentifiedResultPublisher::publish_message()
{
    auto coords = armor_identifier_.get_coordinates_closest();
    identifier::msg::PointData msg;
    msg.leftup = {coords.leftup.x, coords.leftup.y};
    msg.leftdown = {coords.leftdown.x, coords.leftdown.y};
    msg.rightup = {coords.rightup.x, coords.rightup.y};
    msg.rightdown = {coords.rightdown.x, coords.rightdown.y};
    msg.center = {coords.center.x, coords.center.y};
    msg.is_identified = coords.is_identified;
    msg.header.stamp = this->now();
    msg.color.data = coords.color;
    publisher_->publish(msg);
}
void IdentifiedResultPublisher::show_ui()
{
    armor_identifier_.show();
}
void IdentifiedResultPublisher::publish_image()
{
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", armor_identifier_.image_output).toImageMsg();
    image_publisher_->publish(*msg);
}
void IdentifiedResultPublisher::publish_markers()
{
    armor_identifiers_ = armor_identifier_.get_coordinates_all();
    if (!armor_identifiers_.empty())
    {
        int id = 0;
        visualization_msgs::msg::MarkerArray marker_array;
        for (const auto& coord : armor_identifiers_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "camera_link";
            marker.header.stamp = this->now();
            marker.ns = "armor";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.2;
            marker.color.a = 1.0;
            if (coord.color == "red")
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            else if (coord.color == "blue")
            {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            geometry_msgs::msg::Point p;
            std::vector<cv::Point2f> pts = {coord.leftup, coord.rightup, coord.rightdown, coord.leftdown, coord.leftup}; // 闭合
            for (const auto& pt : pts)
            {
                p.x = pt.x;
                p.y = pt.y;
                p.z = 0.0;
                marker.points.push_back(p);
            }
            marker_array.markers.push_back(marker);
        }
        marker_publisher_->publish(marker_array);
    }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IdentifiedResultPublisher>();

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }
    cv::Mat frame;
    while (rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) break;
        node->update_frame(frame);
        node->show_ui();
        node->publish_message();
        node->publish_image();
        node->publish_markers();
        if (cv::waitKey(1) == 27) break;
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}

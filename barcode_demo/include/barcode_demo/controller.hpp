
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "barcode_msgs/msg/scan_result.hpp"
#include "image_draw_msgs/msg/image_marker.hpp"

namespace barcode_demo
{

class controller : public rclcpp::Node
{
public:
    controller(const rclcpp::NodeOptions& options);

private:
    void scan_result_callback(
        const barcode_msgs::msg::ScanResult::SharedPtr scan_result_msg);

    void shutdown_callback();

    rclcpp::Publisher<image_draw_msgs::msg::ImageMarker>::SharedPtr m_marker_pub;
    rclcpp::Subscription<barcode_msgs::msg::ScanResult>::SharedPtr m_scan_result_sub;
};

} // namespace barcode_demo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(barcode_demo::controller)

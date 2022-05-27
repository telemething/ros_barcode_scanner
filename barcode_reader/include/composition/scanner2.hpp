#pragma once

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.h"

#include "barcode_msgs/msg/scan_result.hpp"

#include "zbar.h"

using namespace rclcpp;

namespace barcode_scan
{

class scanner2 : public Node
{
public:
    scanner2(const NodeOptions& options);

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    void shutdown_callback();

    Publisher<barcode_msgs::msg::ScanResult>::SharedPtr m_scan_result_pub;
    image_transport::Subscriber m_image_sub;

    zbar::ImageScanner m_scanner;

    rclcpp::Time last_time_;
    std::chrono::nanoseconds period_;
    double msgs_per_sec_ = 2.0;
};

} // namespace barcode_scan

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(barcode_scan::scanner2)

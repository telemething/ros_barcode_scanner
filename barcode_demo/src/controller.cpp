#include "barcode_demo/controller.hpp"

namespace barcode_demo
{

controller::controller(const rclcpp::NodeOptions& options)
:   rclcpp::Node("controller", options)
{
    rclcpp::on_shutdown([&]
    {
        controller::shutdown_callback();
    });

    using std::placeholders::_1;

    m_marker_pub = this->create_publisher<image_draw_msgs::msg::ImageMarker>("image_markers",
        rclcpp::QoS(10));

    //m_scan_result_sub = this->create_subscription<barcode_msgs::msg::ScanResult>(
    //    "barcodes", rclcpp::QoS(10),
    //    std::bind(&controller::scan_result_callback, this, _1));  
}

void controller::scan_result_callback(
    const barcode_msgs::msg::ScanResult::SharedPtr scan_result_msg)
{
    builtin_interfaces::msg::Duration lifetime;
    lifetime.sec = 0;
    lifetime.nanosec = 500000000; // 500 ms

    std_msgs::msg::ColorRGBA color;
    color.g = 1.0;

    for(const auto& barcode : scan_result_msg->barcodes)
    {
        if(barcode.type != "QR-Code") break;
        
        image_draw_msgs::msg::ImageMarker p_mark;
        p_mark.type = image_draw_msgs::msg::ImageMarker::POLYGON;
        p_mark.ns = "barcode";
        p_mark.id = 0;
        p_mark.lifetime = lifetime;
        p_mark.thickness = 2;
        p_mark.outline_color = color;

        for(const auto& point : barcode.points)
        {
            p_mark.points.push_back(point);
        }

        image_draw_msgs::msg::ImageMarker t_mark;
        t_mark.type = image_draw_msgs::msg::ImageMarker::TEXT;
        t_mark.ns = "text";
        t_mark.id = 0;
        t_mark.lifetime = lifetime;
        t_mark.scale = 1.0;
        t_mark.outline_color = color;
        t_mark.position = barcode.points[0];
        t_mark.text = barcode.data;

        m_marker_pub->publish(p_mark);
        m_marker_pub->publish(t_mark);
    }
}

void controller::shutdown_callback()
{
    
}

} // namespace barcode_demo

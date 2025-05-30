
#include "barcode_scan/scanner.hpp"

#include <functional>

#include "cv_bridge/cv_bridge.h"

namespace barcode_scan
{

scanner::scanner(const NodeOptions& options)
:   Node("scanner", options)
{
    on_shutdown([&]
    {
        scanner::shutdown_callback();
    });

    using std::placeholders::_1;

    m_image_sub = image_transport::create_subscription(this, "image_raw",
        std::bind(&scanner::image_callback, this, _1),
        "raw", rmw_qos_profile_sensor_data);

    m_scan_result_pub = this->create_publisher<barcode_msgs::msg::ScanResult>("barcodes",
        QoS(10));
}

rclcpp::Time last_time_;
std::chrono::nanoseconds period_;

void scanner::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
{
    //****************************

    // TODO : once at init
    double msgs_per_sec_ = 1.0;
    period_ = rclcpp::Rate(msgs_per_sec_).period();

    const auto & now = this->now();

    if (last_time_ > now) 
    {
      //RCLCPP_WARN(
      //  get_logger(), "Detected jump back in time, resetting throttle period to now for.");
      last_time_ = now;
    }

    if ((now - last_time_).nanoseconds() >= period_.count()) 
    {
      //pub_->publish(*msg);
      last_time_ = now;
    }
    else 
    {
      return;
    }

    //****************************

    const cv::Mat cv_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8)->image;

    zbar::Image image(cv_image.cols, cv_image.rows, "Y800", (uchar*)cv_image.data,
        cv_image.cols * cv_image.rows);
    m_scanner.scan(image);

    barcode_msgs::msg::ScanResult scan_result_msg;

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
        barcode_msgs::msg::Barcode barcode;
        barcode.type = symbol->get_type_name();
        barcode.data = symbol->get_data();

        for(int i = 0; i < symbol->get_location_size(); ++i)
        {
            geometry_msgs::msg::Point point;
            point.x = symbol->get_location_x(i);
            point.y = symbol->get_location_y(i);
            barcode.points.push_back(point);
        }

        scan_result_msg.barcodes.push_back(barcode);
    }
    
    if(scan_result_msg.barcodes.size() > 0)
    {
        scan_result_msg.header = image_msg->header;
        m_scan_result_pub->publish(scan_result_msg);
    }
} // scanner::image_callback()

void scanner::shutdown_callback()
{

}

} // namespace barcode_scan

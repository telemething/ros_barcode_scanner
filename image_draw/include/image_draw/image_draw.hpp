#pragma once

#include <map>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"

#include "image_draw_msgs/msg/image_marker.hpp"

using namespace rclcpp;

namespace image_draw
{

class image_draw : public Node
{
public:
    image_draw(const NodeOptions& options);

    static std::string get_marker_id(const image_draw_msgs::msg::ImageMarker& msg);

private:
    typedef image_draw_msgs::msg::ImageMarker marker_t;
    typedef std::pair<marker_t::SharedPtr, Time> marker_pair_t;
    typedef std::unordered_map<std::string, marker_pair_t> marker_map_t;

    static cv::Scalar get_cv_color(const std_msgs::msg::ColorRGBA& color_msg);
    static cv::Point get_cv_point(const geometry_msgs::msg::Point& point_msg);

    void draw(const cv::Mat& image, const marker_t& marker);
    void draw_circle(const cv::Mat& image, const marker_t& marker);
    void draw_line_strip(const cv::Mat& image, const marker_t& marker);
    void draw_line_list(const cv::Mat& image, const marker_t& marker);
    void draw_polygon(const cv::Mat& image, const marker_t& marker);
    void draw_points(const cv::Mat& image, const marker_t& marker);
    void draw_text(const cv::Mat& image, const marker_t& marker);

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    void image_marker_callback(const marker_t::SharedPtr image_marker_msg);
    void shutdown_callback();

    image_transport::Publisher m_image_pub;
    image_transport::Subscriber m_image_sub;
    Subscription<marker_t>::SharedPtr m_image_marker_sub;

    marker_map_t m_marker_map;
    std::mutex m_marker_map_mutex;
};

} // namespace image_draw

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_draw::image_draw)

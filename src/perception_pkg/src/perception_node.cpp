#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

class PerceptionNode : public rclcpp::Node
{

public:
    PerceptionNode() : Node("perception_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image(
            "/rgb/image_raw",
            rclcpp::SensorDataQoS(),
            std:bind(&PerceptionNode::callback, this, std::placeholders::_1)
        )
    }

private:

    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Encoding %s | Width %d | Height %d",
            msg->encoding.c_str(), msg->width, msg-height);
        
        cv_bridge::CvImagePtr cv_ptr;

        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        
        cv::Mat frame = cv_ptr->image;

    }

    rclcpp::Subscription<sensor_msgs>::msg::Image>::SharedPtr sub_; // + ownership


};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
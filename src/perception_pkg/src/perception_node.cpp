#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include <onnxruntime_cxx_api.h>


class PerceptionNode : public rclcpp::Node
{

public:
    PerceptionNode()
     : Node("perception_node"), env_(), session_()
    {


        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&PerceptionNode::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "perception_node up"
        );

        this->declare_parameter<std::string>(
            "model_path",
            "/home/lu/yolo_env/yolo26n.onnx"
        );

        std::string model_path =this->get_parameter("model_path").as_string();


        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);

        
        std::vector<int64_t> shape;
        shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

        for (int64_t d : shape)
        {
            RCLCPP_INFO(this->get_logger(), "shape: %u", d);
        }

        Ort::AllocatorWithDefaultOptions allocator;

        auto in_count = session_->GetInputCount(0, allocator);
        RCLCPP_INFO(this->get_logger(), "input: %s", in_count.get());


        auto out_count = session_->GetOutputCount(0, allocator);
        RCLCPP_INFO(this->get_logger(), "output: %s", out_count.get());
    }

private:

    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Encoding %s | Width %u | Height %u",
            msg->encoding.c_str(), msg->width, msg->height
        );
        
        cv_bridge::CvImagePtr cv_ptr;

        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        
        cv::Mat frame = cv_ptr->image;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Rows: %u, Cols: %u, Channels: %u",
            frame.rows, frame.cols, frame.channels()
        );

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // + ownership


    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
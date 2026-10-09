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
            RCLCPP_INFO(this->get_logger(), "vector elem shape: %ld", d);
        }
        
        Ort::AllocatorWithDefaultOptions allocator;
        auto in_name = session_->GetInputNameAllocated(0, allocator);
        auto out_name = session_->GetOutputNameAllocated(0, allocator);

        RCLCPP_INFO(
            this->get_logger(),
            "input name: %s, output_name: %s ;",
             in_name.get(), out_name.get()
            );
        
        int64_t in_count = session_->GetInputCount();
        int64_t out_count = session_->GetOutputCount();


        RCLCPP_INFO(
            this->get_logger(),
            "input count: %zu, output count: %zu",
            in_count, out_count
        );
        
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&PerceptionNode::callback, this, std::placeholders::_1)
        );
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
            "Rows: %d, Cols: %d, Channels: %d",
            frame.rows, frame.cols, frame.channels()
        );

    }
    
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // + ownership
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
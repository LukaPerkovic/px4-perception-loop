#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

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

        try
        {
        std::string model_path = this->get_parameter("model_path").as_string();
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(0);
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
        }
        catch (const Ort::Exception& e)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Error while finding model path: %s", e.what()
            );

            throw;
        }

        mem = Ort::MemoryInfo::CreateCpu(
            OrtAllocatorType::OrtDeviceAllocator,
            OrtMemType::OrtMemTypeCPU
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

        cv::Mat blob = cv::dnn::blobFromImage(
            frame,
            1.0/255.0,
            cv::Size(640, 640),
            cv::Scalar(),
            true, // swapRB
            false // crop
        );


        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "blob.dims: %d,"
            "blob.size[0]: %d, blob.size[1]: %d, blob.size[2]: %d, blob.size[3]: %d,"
            "blob.total(): %zu,"
            "blob.isContinuous(): %d",
            blob.dims,
            blob.size[0], blob.size[1], blob.size[2], blob.size[3],
            blob.total(),
            blob.isContinuous()
        );
                
        std::vector<int64_t> shape = {1, 3, 640, 640};
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
        

        const char* in_names[] = {"images"};
        const char* out_names[] = {"output0"};

        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            in_names, &input_tensor, 1,
            out_names, 1
        );

        const float* ptr = outputs[0].GetTensorData<float>();
        cv::Mat det(300, 6, CV_32F, (void*)ptr);

        double maxv = 0.0;
        cv::Point maxloc;
        cv::minMaxLoc(det.col(4), nullptr, &ptr, nullptr, &ptr);

        int n = cv::countNonZero(det.col(4) > 0.25);

        const float* best = det.ptr<float>(maxloc.y);

        char buf[256];
        snprintf(buf, sizeof(buf),
            "max_conf=%.4f at row=%d n=%d | box=[%.1f %1.f %.1f %.1f] cls=%.1f",
            best[4], maxloc.y, n
            best[0], best[1], best[2], best[3], best[5]
        );

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            this->get_logger(),
            "%s",
            buf
        );

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "[OUTPUTS]Outputs size: %zu | Outputs shape: %s | Element count: %zu",
        //     outputs.size(), output_shape.c_str(), type_info.GetElementCount()
        // );

    }
    
    Ort::Env env_;

    std::unique_ptr<Ort::Session> session_;

    Ort::MemoryInfo mem{nullptr};
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // + ownership

    bool imgSave_ = false;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
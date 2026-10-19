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

        std::vector<float> input_data(1228800, 0.0f);
        std::vector<int64_t> input_shape = {1, 3, 640, 640};
        


        Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
            mem,
            input_data.data(), input_data.size(),
            input_shape.data(), input_shape.size()
        );

        const char* in_names[] = {"images"};
        const char* out_names[] = {"output0"};

        const auto t1 = std::chrono::steady_clock::now();

        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            in_names, &in_tensor, 1,
            out_names, 1
        );

        const auto t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<
            std::chrono::milliseconds
        >(t2 - t1);
        
        
        auto type_info = outputs.front().GetTensorTypeAndShapeInfo();
        std::string output_shape;
        for (int64_t d : type_info.GetShape())
        {
            output_shape = output_shape + std::to_string(d) + " ";
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Outputs size: %zu | Outputs shape: %s | Element count: %zu | Processing time: %.4f",
            outputs.size(), output_shape.c_str(), type_info.GetElementCount(), time_span.count()
        );


        

        
        // std::vector<int64_t> input_shape;
        // std::vector<int64_t> output_shape;
        // input_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        // output_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

        
        // for (int64_t d : input_shape)
        // {
        //     RCLCPP_INFO(this->get_logger(), "input vector elem shape: %ld", d);
        // }

        // for (int64_t d : output_shape)
        // {
        //     RCLCPP_INFO(this->get_logger(), "output vector elem shape: %ld", d);
        // }
        

        
        // Ort::AllocatorWithDefaultOptions allocator;
        // auto in_name = session_->GetInputNameAllocated(0, allocator);
        // auto out_name = session_->GetOutputNameAllocated(0, allocator);

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "input name: %s, output_name: %s ;",
        //      in_name.get(), out_name.get()
        //     );
        
        // size_t in_count = session_->GetInputCount();
        // size_t out_count = session_->GetOutputCount();


        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "input count: %zu, output count: %zu",
        //     in_count, out_count
        // );

        



        
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
        Ort::Value in = Ort::Value::CreateTensor<float>(
            mem, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
        
        if (imgSave_ == false)
        {
            cv::imwrite("/tmp/frame.png", frame);
            imgSave_ = true;
        }

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
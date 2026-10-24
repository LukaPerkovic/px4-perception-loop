#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <onnxruntime_cxx_api.h>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

struct DetectionView
{
    const float* data;
    int rows;
    int cols;
};


vision_msgs::msg::Detection2DArray toDetections(
    const DetectionView& det_view,
    const std_msgs::msg::Header& header,
    float conf_thresh,
    int src_w, int src_h, float model_w, float model_h)
{
    vision_msgs::msg::Detection2DArray out;
    out.header = header;

    const float x_scale = src_w / model_w;
    const float y_scale = src_h / model_h;

    for (int r = 0; r < det_view.rows; ++r)
    {
        const float* row = det_view.data + r * det_view.cols;

        if (row[4] < conf_thresh) break; // Output is sorted, 'break' exits early on purpose.

        vision_msgs::msg::Detection2D d;
        d.header = header;

        d.bbox.center.position.x = x_scale * (row[0] + row[2]) / 2;
        d.bbox.center.position.y = y_scale * (row[1] + row[3]) / 2;
        d.bbox.center.theta      = 0.0;
        d.bbox.size_x            = x_scale * (row[2] - row[0]);
        d.bbox.size_y            = y_scale * (row[3] - row[1]);

        vision_msgs::msg::ObjectHypothesisWithPose h;
        h.hypothesis.class_id    = std::to_string(std::lround(row[5]));
        h.hypothesis.score       = row[4];
        d.results.push_back(h);

        out.detections.push_back(d);
    }

    return out;
}


class PerceptionNode : public rclcpp::Node
{

public:
    PerceptionNode()
     : Node("perception_node"),
       mem_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator,OrtMemTypeCPU)),
       env_(),
       session_()
    {
        
        int kThreadsNumber {2};
        
        conf_threshold_ = this->declare_parameter<double>(
            "confidence_threshold",
            0.25
        );
        const std::string model_path = this->declare_parameter<std::string>(
            "model_path",
            ""
        );

        if (model_path.empty())
        {
            RCLCPP_FATAL(this->get_logger(),
            "Parameter 'model_path' is not set.");

            throw std::runtime_error("model_path parameter not set");
        }

        if (!std::filesystem::exists(model_path))
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "model_path does not exist: %s",
                model_path.c_str()
            );

            throw std::runtime_error("model file not found: " + model_path);
        }

        try
        {
            Ort::SessionOptions opts;
            opts.SetIntraOpNumThreads(kThreadsNumber);
                    
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
        
        {  
            Ort::AllocatorWithDefaultOptions alloc;
            Ort::AllocatedStringPtr in = session_->GetInputNameAllocated(0, alloc);
            Ort::AllocatedStringPtr out = session_->GetOutputNameAllocated(0, alloc);

            model_input_name_  = in.get();
            model_output_name_ = out.get();
        }

        const auto out_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        
        RCLCPP_INFO(
            this->get_logger(),
            "Supposed rows %ld, supposed cols %ld supposed batch %ld",
            out_shape[0], out_shape[1], out_shape[2]);
           
        // output shape vector is [batch, rows, columns]. We don't need to check batch.
        if (out_shape[1] != 300 || out_shape[2] != 6)
        {
            throw std::runtime_error(
                "Model output not matching requirements."
            );
        }

        std::vector <int64_t> model_input_shape_ = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        // ONNX is NCHW 
        model_input_height_ = model_input_shape_[2];
        model_input_width_ = model_input_shape_[3];
        
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "rgb/image_raw",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {callback(msg);}
        );

        pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/detections", 10);



        RCLCPP_INFO(
            this->get_logger(),
            "perception_node | threads number: %d", kThreadsNumber
        );
    }

private:

    void callback(sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Error while receiving image frames. Dropping the frame -> %s",
            e.what()
            );
            return;
        } 

        cv::Mat frame = cv_ptr->image;

        cv::Mat blob = cv::dnn::blobFromImage(
            frame,
            1.0/255.0,
            cv::Size(model_input_width_, model_input_height_),
            cv::Scalar()
        );
                
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_, blob.ptr<float>(), blob.total(), model_input_shape_.data(), model_input_shape_.size());
        
        const char* in_names[] = { model_input_name_.c_str() };
        const char* out_names[] = { model_output_name_.c_str() };

        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            in_names, &input_tensor, 1,
            out_names, 1
        );

        const float* ptr = outputs[0].GetTensorData<float>();
        
        std::vector <int64_t> output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();

        DetectionView det{ptr, static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2])};



        auto det_msg = toDetections(det,
            msg->header,
            conf_threshold_, msg->width, msg->height,
            model_input_width_, model_input_height_);
        pub_->publish(det_msg);
        
    }

    Ort::MemoryInfo mem_;

    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;

    std::string model_input_name_;
    std::string model_output_name_;
    std::vector<int64_t> model_input_shape_;
    int model_input_height_;
    int model_input_width_;

    double conf_threshold_;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<PerceptionNode>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("perception_node"), "Startup failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
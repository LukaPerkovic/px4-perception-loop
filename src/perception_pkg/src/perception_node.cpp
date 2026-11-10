#include <algorithm>
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

struct LetterboxInfo
{
    float scale;
    int   pad_x;
    int   pad_y;
};


static LetterboxInfo letterbox(const cv::Mat& src,
                               cv::Mat& dst,
                               int net_w,
                               int net_h,
                               const cv::Scalar& pad_color = cv::Scalar(114, 114, 144))
{
    const float scale = std::min(static_cast<float>(net_w) / static_cast<float>(src.cols),
                                 static_cast<float>(net_h) / static_cast<float>(src.rows));

    const int new_w = static_cast<int>(std::lround(src.cols * scale));
    const int new_h = static_cast<int>(std::lround(src.rows * scale));

    const int pad_x = (net_w - new_w) / 2;
    const int pad_y = (net_h - new_h) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    dst.create(net_h, net_w, src.type());
    dst.setTo(pad_color);

    resized.copyTo(dst(cv::Rect(pad_x, pad_y, new_w, new_h)));

    return LetterboxInfo{scale, pad_x, pad_y};

}


vision_msgs::msg::Detection2DArray toDetections(
    const DetectionView& det_view,
    const std_msgs::msg::Header& header,
    float conf_thresh,
    const LetterboxInfo& lb,
    int src_w, int src_h)
{
    vision_msgs::msg::Detection2DArray out;
    out.header = header;

    const float inv_scale = 1.0f / lb.scale;
    const float max_x = static_cast<float>(src_w - 1);
    const float max_y = static_cast<float>(src_h - 1);

    for (int r = 0; r < det_view.rows; ++r)
    {
        const float* row = det_view.data + r * det_view.cols;

        if (row[4] < conf_thresh) break; // Output is sorted, 'break' exits early on purpose.
        
        float x1 = (row[0] - lb.pad_x) * inv_scale;
        float y1 = (row[1] - lb.pad_y) * inv_scale;
        float x2 = (row[2] - lb.pad_x) * inv_scale;
        float y2 = (row[3] - lb.pad_y) * inv_scale;

        x1 = std::clamp(x1, 0.0f, max_x);
        y1 = std::clamp(y1, 0.0f, max_y);
        x2 = std::clamp(x2, 0.0f, max_x);
        y2 = std::clamp(y2, 0.0f, max_y);

        vision_msgs::msg::Detection2D d;
        d.header = header;

        d.bbox.center.position.x = 0.5f * (x1 + x2);
        d.bbox.center.position.y = 0.5f * (y1 + y2);
        d.bbox.center.theta      = 0.0;
        d.bbox.size_x            = x2 - x1;
        d.bbox.size_y            = y2 - y1;

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
       mem_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator,OrtMemTypeCPU))
    {
        
        constexpr int kThreadsNumber {2};
        
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
        
   
        // output shape vector is [batch, rows, columns]. We don't need to check batch.
        if (out_shape[1] != 300 || out_shape[2] != 6)
        {
            throw std::runtime_error(
                "Model output not matching requirements."
            );
        }

        model_input_shape_ = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        // ONNX is NCHW 
        model_input_height_ = model_input_shape_[2];
        model_input_width_ = model_input_shape_[3];
        
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "rgb/image_raw",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {callback(msg);}
        );

        pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);



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

        const LetterboxInfo lb = letterbox(frame, letterboxed_,
                                           model_input_width_, model_input_height_);

        cv::Mat blob = cv::dnn::blobFromImage(
            letterboxed_,
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
            conf_threshold_,
            lb,
            static_cast<int>(msg->width), static_cast<int>(msg->height));
        pub_->publish(det_msg);


        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "frame ok: %zu detections", det_msg.detections.size()    
        );
        
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

    cv::Mat letterboxed_;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
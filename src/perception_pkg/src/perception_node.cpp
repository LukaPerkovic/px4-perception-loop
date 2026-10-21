#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <onnxruntime_cxx_api.h>

#include <vision_msgs/msg/detection2_d_array.hpp>


vision_msgs::msg::Detection2DArray toDetections(
    const cv::Mat& det,
    const std_msgs::msg::Header& header,
    float conf_thresh,
    int src_w, int src_h)
{
    vision_msgs::msg::Detection2DArray out;
    out.header = header;

    const float model_w { 640.0f };
    const float model_h { 640.0f }; 

    const float x_scale = src_w / model_w;
    const float y_scale = src_h / model_h;

    for (int r = 0; r < det.rows; ++r)
    {
        const float* row = det.ptr<float>(r);

        if (row[4] < conf_thresh) break;

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

        pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/detections", 10);
    }

private:

    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        
        cv_bridge::CvImagePtr cv_ptr;

        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        
        cv::Mat frame = cv_ptr->image;

        cv::Mat blob = cv::dnn::blobFromImage(
            frame,
            1.0/255.0,
            cv::Size(640, 640),
            cv::Scalar(),
            true, // swapRB
            false // crop
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
        cv::minMaxLoc(det.col(4), nullptr, &maxv, nullptr, &maxloc);

        auto det_msg = toDetections(det,
            msg->header,
            0.25f, msg->width, msg->height);
        pub_->publish(det_msg);
        
    }

    
    Ort::Env env_;

    std::unique_ptr<Ort::Session> session_;

    Ort::MemoryInfo mem{nullptr};
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; // + ownership
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
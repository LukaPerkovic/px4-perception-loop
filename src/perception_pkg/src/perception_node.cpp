#include <cmath>
#include <functional>
#include <memory>
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
     : Node("perception_node"), env_(), session_()
    {
        
        int THREADS_NUMBER {2};
        
        this->declare_parameter<std::string>(
            "model_path",
            "/home/lu/yolo_env/yolo26n.onnx"
        );
        this->declare_parameter<float>(
            "confidence_threshold",
            0.25f
        )

        
        try
        {
            Ort::SessionOptions opts;
            opts.SetIntraOpNumThreads(THREADS_NUMBER);

            std::string model_path = this->get_parameter("model_path").as_string();
            
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

        mem_ = Ort::MemoryInfo::CreateCpu(
            OrtAllocatorType::OrtDeviceAllocator,
            OrtMemType::OrtMemTypeCPU
        );
        
        Ort::AllocatorWithDefaultOptions alloc;
        const auto input_name = session->GenInputNameAllocated(0, alloc);
        const auto output_name = session->GetOutputNameAllocated(0, alloc);

        std::vector <int64_t> model_input_shape = session_.GetInputTypeAndShapeInfo().GetShape();
        const int64_t model_input_width = model_input_shape[2];
        const int64_t model_input_height = model_input_shape[3];
        
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb/image_raw",
            rclcpp::SensorDataQoS(),
            [this](Image::ConstSharedPtr msg) {callback(msg);}
        );

        pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/detections", 10);



        RCLCPP_INFO(
            this->get_logger(),
            "perception_node up | model_path: %s | input_name: %s | output_name: %s | threads number: %d",
            model_path, input_name, output_name, THREADS_NUMBER
        );
    }

private:

    void callback(sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (const cv_bridge::Exception&)
        {
            RCLCPP_FATAL_THROTTLED(
            this->get_logger(),
            this->get_clock(),
            1000,
            "Error while finding model path: %s", e.what()
            );
            return;
        } 

        cv::Mat frame = cv_ptr->image;

        cv::Mat blob = cv::dnn::blobFromImage(
            frame,
            1.0/255.0,
            cv::Size(model_input_width, model_input_height),
            cv::Scalar(),
            true, // swapRB
            false // crop
        );
                
        std::vector<int64_t> input_shape = {1, 3, 640, 640};
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
        

        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            input_name, &input_tensor, 1,
            output_name, 1
        );

        const float* ptr = outputs[0].GetTensorData<float>();
        
        std::vector <int64_t> output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();

        // output shape vector is [batch, rows, columns]. We don't need to check batch.
        if (output_shape[1] != 300 || output_shape[2] != 6)
        {
            throw std::runtime_error(
                "Model output not matching requiremnts."
                "Required/Expectted: Rows 300/%d | Columns: 6/%d",
                outpt_shape[1], output_shape[2]
            );
        }

        cv::Mat det(300, 6, CV_32F, (void*)ptr);

        float conf_threshold = this->get_parameter("confidence_threshold");


        auto det_msg = toDetections(det,
            msg->header,
            conf_threshold, msg->width, msg->height);
        pub_->publish(det_msg);
        
    }

    
    Ort::Env env_;

    std::unique_ptr<Ort::Session> session_;

    Ort::MemoryInfo mem_{nullptr};
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
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
        const float* row = det_view.ptr<float>(r);

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
       mem_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator,OrtMemTypeCPU))
    {
        
        int kThreadsNumber {2};
        
        std::string model_path;

        this->declare_parameter<std::string>(
            "model_path",
            "/home/lu/yolo_env/yolo26n.onnx"
        );
        this->declare_parameter<double>(
            "confidence_threshold",
            0.25
        ).as_double();

        try
        {
            Ort::SessionOptions opts;
            opts.SetIntraOpNumThreads(THREADS_NUMBER);

            model_path = this->get_parameter("model_path").as_string();
            
            session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
        }
        catch (const Ort::Exception& e)
        {   
            RCLCPP_FATAL(
                this->get_logger(),
                "Error while finding model path: %s", e.what()
            );
            model_path = "invalid";
            
            throw;
        }
;
        
        Ort::AllocatorWithDefaultOptions alloc;
        mode_input_name_ = session_->GetInputNameAllocated(0, alloc);
        model_output_name_ = session_->GetOutputNameAllocated(0, alloc);

        std::vector <int64_t> model_input_shape = sesion_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        // ONNX is NCHW 
        model_input_height_ = model_input_shape[2];
        model_input_width_ = model_input_shape[3];
        
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "rgb/image_raw",
            rclcpp::SensorDataQoS(),
            [this](Image::ConstSharedPtr msg) {callback(msg);}
        );

        pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/detections", 10);



        RCLCPP_INFO(
            this->get_logger(),
            "perception_node up input_name: %s | output_name: %s | threads number: %d",
            model_input_name_, model_output_name_, kThreadsNumber
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
            RCLCPP_WARN_THROTTLED(
            this->get_logger(),
            this->get_clock(),
            1000,
            "Error while receiving image frames. Dropping the frame."
            );
            return;
        } 

        cv::Mat frame = cv_ptr->image;

        cv::Mat blob = cv::dnn::blobFromImage(
            frame,
            1.0/255.0,
            cv::Size(model_input_width, model_input_height),
            cv::Scalar(),
            false // crop
        );
                
        std::vector<int64_t> input_shape = {1, 3, 640, 640};
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_, blob.ptr<float>(), blob.total(), input_shape.data(), input_shape.size());
        

        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            model_input_name_, &input_tensor, 1,
            model_output_name, 1
        );

        const float* ptr = outputs[0].GetTensorData<float>();
        
        std::vector <int64_t> output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();

        // output shape vector is [batch, rows, columns]. We don't need to check batch.
        if (output_shape[1] != 300 || output_shape[2] != 6)
        {
            throw std::runtime_error(
                "Model output not matching requirements."
                "Required/Expectted: Rows 300/%d | Columns: 6/%d",
                outpt_shape[1], output_shape[2]
            );
        }

        DetectionView det(ptr, output_shape[1], output_shape[2]);

        float conf_threshold = this->get_parameter("confidence_threshold");


        auto det_msg = toDetections(det,
            msg->header,
            conf_threshold, msg->width, msg->height,
            model_input_width_, model_input_height_);
        pub_->publish(det_msg);
        
    }

    
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;

    const char* model_input_name_;
    const char* model_output_name_;
    const int64_t model_input_height_;
    const int64_t model_input_width_;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
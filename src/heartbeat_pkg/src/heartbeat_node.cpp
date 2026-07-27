#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/u_int64.hpp>

using namespace std::chrono_literals;

class HeartbeatNode : public rclcpp::Node
{
public:
    HeartbeatNode() : Node("heartbeat_node")
    {

       auto qos = rclcpp::SensorDataQoS();

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            qos,
            std::bind(&HeartbeatNode::position_callback, this, std::placeholders::_1)
         );

        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("uav_heartbeat", 10);

        timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&HeartbeatNode::publish_heartbeat, this)
        );



        RCLCPP_INFO(this->get_logger(), "heartbeat_node started.");
        
    }
private:
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "z: %.2f", msg->z);
    }

    void publish_heartbeat()
    {
        auto msg = std_msgs::msg::UInt64();
        msg.data = m_counter++;
        publisher_->publish(msg);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
    uint64_t m_counter{0};

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeartbeatNode>());
    rclcpp::shutdown();
    return 0;
}
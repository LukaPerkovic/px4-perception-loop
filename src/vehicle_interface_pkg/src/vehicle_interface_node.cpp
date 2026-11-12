#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace std::chrono_literals;

class VehicleInterfaceNode : public rclcpp::Node
{
public:
    VehicleInterfaceNode() : Node("vehicle_interface_node")
    {

        auto qos = rclcpp::SensorDataQoS();

        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/offboard_control_mode", qos);
        
        timer_ = this->create_wall_timer(
            150ms,
            [this]() {VehicleInterfaceNode::publish_offboard_control_mode});
        
        RCLCPP_INFO(this->get_logger(), "vehicle_interface_node up.");
    }


private:
    void publish_offboard_control_mode()
    {

        px4_msgs::msg::OffboardControlMode msg{};

        msg.position     = _____;
        msg.velocity     = _____;
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;
        msg.timestamp    = ______;
        offboard_mode_pub_->publish(msg);
    }

    rclcpp::Publisher<_____>::SharedPtr offboard_mode_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleInterfaceNode>());
    rlcpp:shutdown();
    return 0;
}
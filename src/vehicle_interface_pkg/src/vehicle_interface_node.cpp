#include <chrono>
#include <numbers>

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
            "/fmu/in/offboard_control_mode", qos);
        
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos);
        
        timer_ = this->create_wall_timer(
            75ms,
            [this]() {
                VehicleInterfaceNode::publish_offboard_control_mode();
                VehicleInterfaceNode::publish_trajectory_setpoint();
            });
        
        RCLCPP_INFO(this->get_logger(), "vehicle_interface_node up.");
    }


private:
    void publish_offboard_control_mode()
    {

        px4_msgs::msg::OffboardControlMode msg{};

        msg.position     = true;
        msg.velocity     = true;
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;
        msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
        offboard_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        msg.position = {0.0f, 0.0f, -2.0f};
        msg.yaw = std::numbers::pi_v<float> / 2.0f;
        msg.timestmap = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
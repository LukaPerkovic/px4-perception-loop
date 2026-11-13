#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

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


        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v4",
            qos,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
                arming_state_ = msg->arming_state;
                nav_state_ = msg->nav_state;
            }
        );


        
        timer_ = this->create_wall_timer(
            75ms,
            [this]() {
                const uint64_t ts = this->get_clock()->now().nanoseconds() / 1000;
                publish_offboard_control_mode(ts);
                publish_trajectory_setpoint(ts);
            }
        );

        RCLCPP_INFO(this->get_logger(), "ARMING STATE: %d | NAV STATE: %d", arming_state_, nav_state_); // temp

        RCLCPP_INFO(this->get_logger(), "vehicle_interface_node up.");

    }


private:
    void publish_offboard_control_mode(const uint64_t timestamp)
    {

        px4_msgs::msg::OffboardControlMode msg{};

        msg.position     = true;
        msg.velocity     = false;
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;
        msg.timestamp    = timestamp;
        offboard_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(const uint64_t timestamp)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        msg.position = {0.0f, 0.0f, -2.0f};
        msg.yaw = NAN;
        msg.timestamp = timestamp;
        trajectory_setpoint_pub_->publish(msg);
    }


    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

    int arming_state_;
    int nav_state_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
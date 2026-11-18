#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
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

        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos);


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
                reconcile(ts);

            }
        );


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

    void publish_vehicle_command(uint16_t command, float param1, float param2, uint64_t timestamp)
    {

        px4_msgs::msg::VehicleCommand msg{};

        msg.command          = command;
        msg.param1           = param1;
        msg.param2           = param2;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external     = true;
        msg.timestamp        = timestamp;

        vehicle_command_pub_->publish(msg);

    }
    
    void reconcile(uint64_t timestamp)
    {
        ++tick_;

        uint64_t tick_limit { 30000 }; // approximate number of 75 microsecond ticks per 2 seconds

        if (tick_ < tick_limit) return;                  // warm-up; ~2 secs

        if (tick_ - last_cmd_tick_ < tick_limit) return; // cooldown

        if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                1.0f, 6.0f, timestamp);
            last_cmd_tick_ = tick_;
            RCLCPP_INFO(this->get_logger(), "requesting OFFBOARD");
            
            return;
        } 

        if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0f, 0.0f, timestamp);
            RCLCPP_INFO(this->get_logger(), "requsting ARM");
        }
    }



    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    uint8_t arming_state_ {0};
    uint8_t nav_state_ {0};

    uint64_t tick_ {0};
    uint64_t last_cmd_tick_ {0};

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "vehicle_interface_pkg/vehicle_interface.hpp"
#include "uav_interfaces/msg/target_intent.hpp"

using namespace std::chrono_literals;

static LinkGoal goal_for_state(uint8_t s)
{
    using TI = uav_interfaces::msg::TargetIntent;
    switch (s) {
        case TI::STATE_SEARCH:    return LinkGoal::FLYING;
        case TI::STATE_TRACK:     return LinkGoal::FLYING;
        case TI::STATE_REACQUIRE: return LinkGoal::FLYING;
        case TI::STATE_LAND:      return LinkGoal::LANDING;
        default:                  return LinkGoal::IDLE;
    }
}


class VehicleInterfaceNode : public rclcpp::Node
{
public:
    VehicleInterfaceNode() : Node("vehicle_interface_node")
    {


        takeoff_alt_m_ = declare_parameter<double>("takeoff_alt_m", 2.0);
        auto_recover_  = declare_parameter<bool>("auto_recover_from_failsafe", false);

        RCLCPP_INFO(get_logger(), "params: takoff_alt=%.2f m auto_recover=%s",
                        takeoff_alt_m_, auto_recover_ ? "true": "false");

        auto qos = rclcpp::SensorDataQoS();

        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos);
        
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos);

        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10); // RELIABLE since it's edge-triggered


        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v4",
            qos,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {

                status_seen_ = true;
                status_watch_.mark_seen(tick_);

                if (msg->arming_state != arming_state_ || msg->nav_state != nav_state_) {
                    RCLCPP_INFO(get_logger(),
                            "PX4: arming %d->%d nav %d->%d",
                            arming_state_, msg->arming_state,
                            nav_state_, msg->nav_state);
                }
                arming_state_ = msg->arming_state;
                nav_state_    = msg->nav_state;
                failsafe_     = msg->failsafe;
                }
            );

        intent_sub_ = create_subscription<uav_interfaces::msg::TargetIntent>(
            "target_intent", 10,
            [this](uav_interfaces::msg::TargetIntent::ConstSharedPtr msg) {
                intent_seen_  = true;
                intent_watch_.mark_seen(tick_);
                set_goal(goal_for_state(msg->state));
            }
        );

        pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
                on_local_position(msg);
            }
        );

        

        timer_ = this->create_wall_timer(kTimerPeriod, [this](){ on_timer(); });


        RCLCPP_INFO(this->get_logger(), "vehicle_interface_node up.");

    }


private:

    // approximate number of 75 milisecond ticks per 2 seconds
    static constexpr auto     kTimerPeriod        = 75ms;
    static constexpr float    kAirborneAltM       = 1.0F;
    static constexpr uint64_t kWarmupTicks        = 27;
    static constexpr uint64_t kCmdCooldownTicks   = 27;
    static constexpr uint64_t kIntentTimeoutTicks = 40;  // ~3s at 13.3h
    static constexpr uint64_t kStatusStaleTicks   = 28;
    static constexpr uint64_t kRecoverTicks       = 27;

    void on_timer()
    {
        ++tick_;
        const uint64_t ts = get_clock()->now().nanoseconds() / 1000; // microseconds

        if (!hold_.has_value() && local_pos_.has_value()) {
            hold_ = *local_pos_;
        }

        if (hold_.has_value())
        {
            publish_offboard_control_mode(ts);
            publish_trajectory_setpoint(*hold_, ts);
        }

        const Freshness fs = status_watch_.check(tick_, kStatusStaleTicks);
        const bool stale = (fs == Freshness::STALE);

        if (stale && !status_blind_)
        {
            status_blind_ = true;
            RCLCPP_ERROR(get_logger(), "vehicle_status stale (%lu ticks) - going hands-off",
                static_cast<unsigned long>(tick_ - *status_watch_.last_seen_tick));
        } else if (!stale && status_blind_) {
            status_blind_ = false;
            RCLCPP_WARN(get_logger(), "vehicle_status recovered");
        }

        const Freshness fi = intent_watch_.check(tick_, kIntentTimeoutTicks);
        if (intent_seen_ &&
            fi == Freshness::STALE &&
            goal_ == LinkGoal::FLYING)
        {
            RCLCPP_WARN(get_logger(), "intent_stale - landing");
            set_goal(LinkGoal::LANDING);
        }

        if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED
                && local_pos_.has_value()) {
                    ground_z_ = local_pos_->z;
                }

        reconcile(ts);
    
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "tick=%lu goal=%s arming=%d nav=%d failsafe=%d",
             tick_, to_string(goal_), arming_state_, nav_state_, (int)failsafe_);

    }

    void on_local_position(px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg)
    {
        if (msg->xy_valid && msg->z_valid) {
            local_pos_ = NedPoint{msg->x, msg->y, msg->z};
            pos_watch_.mark_seen(tick_);
        }
    }

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

    void publish_trajectory_setpoint(const NedPoint& point, const uint64_t timestamp)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        msg.position = {point.x, point.y, point.z};
        msg.yaw = NAN;
        msg.timestamp = timestamp;

        trajectory_setpoint_pub_->publish(msg);
    }

    void set_goal(LinkGoal g)
    {
        if (surrendered_ && g != LinkGoal::IDLE) { return; }
        if (g == goal_) return;

        if (local_pos_.has_value()) {
            hold_ = (g == LinkGoal::FLYING)
                ? NedPoint{
                    local_pos_->x,
                    local_pos_->y,
                    local_pos_->z - static_cast<float>(takeoff_alt_m_)}
                : *local_pos_;

            RCLCPP_INFO(get_logger(), "hold point captured: %.2f %.2f %.2f",
                hold_->x, hold_->y, hold_->z);
        }

        RCLCPP_INFO(get_logger(), "goal %s -> %s", to_string(goal_), to_string(g));
        goal_ = g;
        last_cmd_tick_ = 0;
    }

    void reconcile(uint64_t timestamp)
    {
        
        if (!status_seen_ || status_blind_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "no VehicleStatus - not commanding");
            return;
        }

        if (!local_pos_.has_value()) { return; }    
        
        if (failsafe_)
        {
            last_failsafe_tick_ = tick_;
            if (!surrendered_)
            {
                surrendered_ = true;
                RCLCPP_WARN(get_logger(), "PX4 failsafe active, surrendering control");
                set_goal(LinkGoal::IDLE);
            }
        }

        if (surrendered_)
        {
            const bool clean = !failsafe_ && (tick_ - last_failsafe_tick_) >= kRecoverTicks;
            const bool airborne_now = airborne();

            if (auto_recover_ && clean && airborne_now)
            {
                surrendered_ = false;
                RCLCPP_WARN(get_logger(), "failsafe cleared, still armed - resuming");
            } else {
                return;
            }
        }

        if (tick_ < kWarmupTicks) return;                       // warm-up; ~2 secs
        if (tick_ - last_cmd_tick_ < kCmdCooldownTicks) return; // cooldown

        switch (goal_) {
        case LinkGoal::IDLE:
            break;

        case LinkGoal::FLYING:
            if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
                send_cmd(
                    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                    1.0f, 6.0f, timestamp, "request OFFBOARD");

            } else if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            send_cmd(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0f, 0.0f, timestamp, "request ARM");
            }
            break;
        
        case LinkGoal::LANDING:
            if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
                RCLCPP_INFO(get_logger(), "landed and disarmed");
                set_goal(LinkGoal::IDLE);
            } else if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
                send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
                0.0f, 0.0f, timestamp, "request LAND");
            }
            break;
 
        }
    }

    void send_cmd(uint16_t command, float param1, float param2, uint64_t timestamp, const char * what)
    {
        publish_vehicle_command(command, param1, param2, timestamp);
        last_cmd_tick_ = tick_;
        RCLCPP_INFO(get_logger(), "%s", what);
    }

    bool airborne() const
    {
        if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) { return false; }
        if (!ground_z_.has_value() || !local_pos_.has_value())                 { return false; }
        return (*ground_z_ - local_pos_->z) > kAirborneAltM;
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

    // --- Observed ---
    uint8_t arming_state_              {0};
    uint8_t nav_state_                 {0};
    bool    status_seen_               {false};
    bool    failsafe_                  {false};
    std::optional<NedPoint> local_pos_ {};
    std::optional<float>    ground_z_  {};

    // --- Desired ---
    LinkGoal                goal_ {LinkGoal::IDLE};
    std::optional<NedPoint> hold_ {};

    // --- Bookkeeping ---
    bool intent_seen_  {false};
    bool status_blind_ {false};
    bool surrendered_  {false};
    
    uint64_t tick_               {0};
    uint64_t last_cmd_tick_      {0};
    uint64_t last_failsafe_tick_ {0};

    StreamWatch status_watch_;
    StreamWatch intent_watch_;
    StreamWatch pos_watch_;
    
    // --- Parameters ---
    bool   auto_recover_ {false};
    double takeoff_alt_m_ {2.0};

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        status_sub_;
    rclcpp::Subscription<uav_interfaces::msg::TargetIntent>::SharedPtr   intent_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr     offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr      trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr          vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr                                         timer_;

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
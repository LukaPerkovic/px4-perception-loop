#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "behavior_pkg/behavior.hpp"
#include "uav_interfaces/msg/target_intent.hpp"

using namespace std::chrono_literals;

static uint8_t to_msg_state(State s)
{
    using TI = uav_interfaces::msg::TargetIntent;
    switch (s)
    {
    case State::SEARCH: return TI::STATE_SEARCH;
    case State::TRACK:  return TI::STATE_TRACK;
    }

    return TI::STATE_UNKNOWN;
}


class BehaviorNode : public rclcpp::Node
{
public:
    BehaviorNode()
     : Node("behavior_node"),
       filter_(3, 5)
    {
        intent_pub_ = create_publisher<uav_interfaces::msg::TargetIntent>("target_intent", 10);
        intent_timer_ = create_wall_timer(kIntentPeriod, [this]() { publish_intent(); });

        sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
            "detections",
            10, // plain RELIABLE/VOLATILE
            [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {on_detections(msg);}
        );

        RCLCPP_INFO(get_logger(), "behavior_node up");
    }

private:

    static constexpr auto kIntentPeriod = 100ms;

    void publish_intent()
    {
        uav_interfaces::msg::TargetIntent msg;
        msg.header.stamp = now();
        msg.state        = to_msg_state(state_);
        msg.target_valid = false;
        intent_pub_->publish(msg);
    }

    bool frame_has_target(const vision_msgs::msg::Detection2DArray& msg) const
    {
        for (const auto& d : msg.detections) {
            if (d.results.empty()) { continue; }
            const auto& h = d.results[0].hypothesis;
            if (h.score >= conf_min_ && h.class_id == target_class_) { return true; }
        }

        return false;
    }

    void on_detections(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
    {
        const bool valid = frame_has_target(*msg);
        const Event e    = filter_.check_validity(valid);
        const State next = next_state(state_, e);

        if (next != state_) {
            RCLCPP_INFO(get_logger(), "state %s -> %s", to_string(state_), to_string(next));
        }

        state_ = next;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "valid=%d state=%s", valid, to_string(state_)
        );
    }

    FrameFilter filter_;
    State       state_ { State::SEARCH };
    double      conf_min_ { 0.6 };
    std::string target_class_ {"0"};

    rclcpp::Publisher<uav_interfaces::msg::TargetIntent>::SharedPtr     intent_pub_;
    rclcpp::TimerBase::SharedPtr                                        intent_timer_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;


};



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorNode>());
    rclcpp::shutdown();
    return 0;
}

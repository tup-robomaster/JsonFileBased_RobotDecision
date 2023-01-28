#include "./robot_decision/RobotDecision.h"

#include "robot_interface/msg/decision.hpp"

namespace robotdecision
{
    class RobotDecisionNode : public rclcpp::Node
    {
        typedef robot_interface::msg::Decision DecisionMsg;

    private:
        // message_filters::Subscriber
        rclcpp::Publisher<DecisionMsg>::SharedPtr _decision_pub;

        std::unique_ptr<RobotDecision> myRD;

    public:
        RobotDecisionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~RobotDecisionNode();

    public:
        void process_once();
    };
}
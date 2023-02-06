#include "./robot_decision/RobotDecision.h"

#include "robot_interface/msg/decision.hpp"

namespace rdsys
{
    class RobotDecisionNode : public rclcpp::Node
    {
        typedef robot_interface::msg::Decision DecisionMsg;

    private:
        // message_filters::Subscriber
        rclcpp::Publisher<DecisionMsg>::SharedPtr _decision_pub;

        std::unique_ptr<RobotDecisionSys> myRD;

    public:
        RobotDecisionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~RobotDecisionNode();

    public:
        void init(char *waypointsPath, char *decisionsPath);
        void process_once(int _HP, int mode, float _x, float _y, int time, std::vector<RobotPosition> friendPositions, std::vector<RobotPosition> enemyPositions);
    };
}
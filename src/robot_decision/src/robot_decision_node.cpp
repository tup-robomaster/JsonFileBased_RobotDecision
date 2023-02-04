#include "../include/robot_decision_node.hpp"

using namespace std::placeholders;
namespace robotdecision
{
    RobotDecisionNode::RobotDecisionNode(const rclcpp::NodeOptions &options)
        : Node("robot_decision", options)
    {
        RCLCPP_WARN(this->get_logger(), "RobotDecision node...");
        rclcpp::QoS qos(0);
        qos.keep_last(10);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        qos.durability_volatile();

        this->_decision_pub = this->create_publisher<DecisionMsg>("/robot_decision/decision_info", qos);
    }

    RobotDecisionNode::~RobotDecisionNode()
    {
    }

    void RobotDecisionNode::process_once(int _HP, int mode, float _x, float _y, int time, std::vector<RobotPosition> friendPositions, std::vector<RobotPosition> enemyPositions)
    {
        int myWayPointID = this->myRD->checkNowWayPoint(_x, _y);
        Decision myDecision = this->myRD->decide(myWayPointID, mode, _HP, time, friendPositions, enemyPositions);
        DecisionMsg msg;
        msg.set__mode(myDecision.decide_mode);
        WayPoint aimWayPoint = this->myRD->getWayPointByID(myDecision.decide_wayPoint);
        msg.set__x(aimWayPoint.x);
        msg.set__y(aimWayPoint.y);
        this->_decision_pub->publish(msg);
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robotdecision::RobotDecisionNode>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robotdecision::RobotDecisionNode)
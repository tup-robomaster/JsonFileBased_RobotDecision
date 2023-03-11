#include "../include/robot_decision_node.hpp"

using namespace std::placeholders;

namespace rdsys
{
    RobotDecisionNode::RobotDecisionNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("robot_decision", options)
    {
        RCLCPP_WARN(this->get_logger(), "RobotDecision node...");
        RCLCPP_WARN(this->get_logger(), "starting...");
        this->init(WayPointsPATH, DecisionsPATH);
    }

    RobotDecisionNode::~RobotDecisionNode()
    {
    }

    void RobotDecisionNode::init(char *waypointsPath, char *decisionsPath)
    {
        if (!this->myRDS->decodeWayPoints(waypointsPath))
            RCLCPP_ERROR(this->get_logger(), "decode waypoints failed");
        if (!this->myRDS->decodeDecisions(decisionsPath))
            RCLCPP_ERROR(this->get_logger(), "decode decisions failed");

        // rclcpp::QoS qos(0);
        // qos.keep_last(10);
        // qos.best_effort();
        // qos.reliable();
        // qos.durability();
        // qos.durability_volatile();

        RCLCPP_INFO(this->get_logger(), "Starting action_client");
        this->action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");
        if (!action_client->wait_for_action_server(std::chrono::seconds(20)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        this->carHP_sub_.subscribe(this, "/car_hp", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        this->carPos_sub_.subscribe(this, "/car_pos", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        this->gameInfo_sub_.subscribe(this, "/game_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        this->sentry_sub_.subscribe(this, "/sentry_msg", rclcpp::SensorDataQoS().get_rmw_qos_profile());

        this->TS_sync_.connectInput(this->carHP_sub_, this->carPos_sub_, this->gameInfo_sub_, this->sentry_sub_);
        this->TS_sync_.registerCallback(RobotDecisionNode::messageCallBack);
    }

    void RobotDecisionNode::process_once(int _HP, int mode, float _x, float _y, int time, std::vector<RobotPosition> friendPositions, std::vector<RobotPosition> enemyPositions)
    {
        int myWayPointID = this->myRDS->checkNowWayPoint(_x, _y);
        Decision *myDecision = this->myRDS->decide(myWayPointID, mode, _HP, time, friendPositions, enemyPositions);
        WayPoint *aimWayPoint = this->myRDS->getWayPointByID(myDecision->decide_wayPoint);
        if (aimWayPoint == nullptr)
            return;
    }

    void RobotDecisionNode::makeNewGoal(double x, double y, double theta)
    {
        auto pose = geometry_msgs::msg::PoseStamped();

        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = 'Robot_Goal';
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);

        acummulated_poses_.emplace_back(pose);
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<rdsys::RobotDecisionNode>();

    // auto nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();
    if (rclcpp::ok())
    {
        rclcpp::spin(my_node);
    }
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rdsys::RobotDecisionNode)
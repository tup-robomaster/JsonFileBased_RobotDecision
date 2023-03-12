#include "../include/robot_decision_node.hpp"

using namespace std::placeholders;

namespace rdsys
{
    RobotDecisionNode::RobotDecisionNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("robot_decision_node", options)
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
        this->declare_parameter<float>("distance_thr", 0.5);
        this->declare_parameter<float>("seek_thr", 5.0);
        this->myRDS = std::make_shared<RobotDecisionSys>(RobotDecisionSys(this->_distance_THR, this->_seek_THR));
        this->timer_ = this->create_wall_timer(1000ms, std::bind(&RobotDecisionNode::respond, this));
        if (!this->myRDS->decodeWayPoints(waypointsPath))
            RCLCPP_ERROR(this->get_logger(), "Decode waypoints failed");
        if (!this->myRDS->decodeDecisions(decisionsPath))
            RCLCPP_ERROR(this->get_logger(), "Decode decisions failed");
        // rclcpp::QoS qos(0);
        // qos.keep_last(10);
        // qos.best_effort();
        // qos.reliable();
        // qos.durability();
        // qos.durability_volatile();

        RCLCPP_INFO(this->get_logger(), "Starting action_client");
        this->nav_through_poses_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");
        if (!nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(20)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        this->carHP_sub_.subscribe(this, "/car_hp", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        this->carPos_sub_.subscribe(this, "/car_pos", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        this->gameInfo_sub_.subscribe(this, "/game_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        this->sentry_sub_.subscribe(this, "/sentry_msg", rclcpp::SensorDataQoS().get_rmw_qos_profile());

        this->TS_sync_.reset(new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(10), this->carHP_sub_, this->carPos_sub_, this->gameInfo_sub_, this->sentry_sub_));
        this->TS_sync_->registerCallback(std::bind(&RobotDecisionNode::messageCallBack, this, _1, _2, _3, _4));
    }

    bool RobotDecisionNode::process_once(int &_HP, int &mode, float &_x, float &_y, int &time, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions)
    {
        acummulated_poses_.clear();
        int myWayPointID = this->myRDS->checkNowWayPoint(_x, _y);
        Decision *myDecision = this->myRDS->decide(myWayPointID, mode, _HP, time, friendPositions, enemyPositions);
        WayPoint *aimWayPoint = this->myRDS->getWayPointByID(myDecision->decide_wayPoint);
        if (aimWayPoint == nullptr)
            return false;
        double theta = this->myRDS->decideAngleByEnemyPos(aimWayPoint->x, aimWayPoint->y, enemyPositions);
        if (theta == -1)
            theta = aimWayPoint->theta;
        this->makeNewGoal(aimWayPoint->x, aimWayPoint->y, theta);
        this->nav_through_poses_goal_.poses = acummulated_poses_;
        RCLCPP_DEBUG(
            this->get_logger(), "Sending a path of %zu waypoints:",
            this->nav_through_poses_goal_.poses.size());
        for (auto waypoint : this->nav_through_poses_goal_.poses)
        {
            RCLCPP_DEBUG(
                this->get_logger(),
                "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
        }
        auto send_goal_options =
            rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto)
        {
            nav_through_poses_goal_handle_.reset();
        };

        auto future_goal_handle =
            nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle, server_timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
            return false;
        }

        nav_through_poses_goal_handle_ = future_goal_handle.get();
        if (!nav_through_poses_goal_handle_)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }
        return true;
    }

    void RobotDecisionNode::makeNewGoal(double x, double y, double &theta)
    {
        auto pose = geometry_msgs::msg::PoseStamped();

        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "Robot_Goal";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);

        acummulated_poses_.emplace_back(pose);
    }

    std::vector<RobotPosition> RobotDecisionNode::point2f2Position(std::array<robot_interface::msg::Point2f, 10UL> pos)
    {
        if (pos.size() != 10)
        {
            RCLCPP_ERROR(this->get_logger(), "Position msg not valid !");
            return {};
        }
        std::vector<RobotPosition> result;
        for (int i = 0; i < int(pos.size()); ++i)
        {
            result.emplace_back(RobotPosition(i, pos[i].x, pos[i].y));
        }
        return result;
    }

    void RobotDecisionNode::messageCallBack(const std::shared_ptr<robot_interface::msg::CarHP const> &carHP_msg_, const std::shared_ptr<robot_interface::msg::CarPos const> &carPos_msg_, const std::shared_ptr<robot_interface::msg::GameInfo const> &gameInfo_msg_, const std::shared_ptr<robot_interface::msg::Sentry const> &sentry_msg_)
    {
        this->nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();
        int myHP = carHP_msg_->hp[this->_selfIndex];
        float myPos_x_ = carPos_msg_->pos[this->_selfIndex].x;
        float myPos_y_ = carPos_msg_->pos[this->_selfIndex].y;
        int nowTime = gameInfo_msg_->timestamp;
        int mode = sentry_msg_->mode;
        std::vector<RobotPosition> allPositions = this->point2f2Position(carPos_msg_->pos);
        std::vector<RobotPosition> friendPositions;
        std::vector<RobotPosition> enemyPositions;
        for (int i = 0; i < 9; ++i)
        {
            if (i < 5)
            {
                if (this->_IsRed)
                    enemyPositions.emplace_back(allPositions[i]);
                else
                    friendPositions.emplace_back(allPositions[i]);
            }
            else
            {
                if (this->_IsRed)
                    friendPositions.emplace_back(allPositions[i]);
                else
                    enemyPositions.emplace_back(allPositions[i]);
            }
        }
        if (this->process_once(myHP, mode, myPos_x_, myPos_y_, nowTime, friendPositions, enemyPositions))
        {
            RCLCPP_ERROR(this->get_logger(), "Decision failed!");
        }
    }

    void RobotDecisionNode::respond()
    {
        this->get_parameter("distance_thr", this->_distance_THR);
        this->get_parameter("seek_thr", this->_seek_THR);

        if (this->myRDS->getDistanceTHR() != this->_distance_THR)
        {
            RCLCPP_INFO(this->get_logger(), "set _distance_THR to %f", this->_distance_THR);
            this->myRDS->setDistanceTHR(this->_distance_THR);
        }
        if (this->myRDS->getSeekTHR() != this->_seek_THR)
        {
            RCLCPP_INFO(this->get_logger(), "set _seek_THR to %f", this->_seek_THR);
            this->myRDS->setSeekTHR(this->_seek_THR);
        }
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<rdsys::RobotDecisionNode>();
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rdsys::RobotDecisionNode)
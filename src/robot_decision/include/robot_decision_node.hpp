#include "./robot_decision/RobotDecision.h"

#include "robot_interface/msg/car_hp.hpp"
#include "robot_interface/msg/car_pos.hpp"
#include "robot_interface/msg/game_info.hpp"
#include "robot_interface/msg/serial.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_behavior_tree/plugins/action/navigate_through_poses_action.hpp"

#include <message_filters/subscriber.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace rdsys
{
    using namespace std::chrono_literals;

    class RobotDecisionNode : public rclcpp::Node
    {
    private:
        int _selfIndex = 0;
        bool _IsRed = false;
        float _distance_THR = 0.5;
        float _seek_THR = 10;

    private:
        message_filters::Subscriber<robot_interface::msg::CarHP> carHP_sub_;
        message_filters::Subscriber<robot_interface::msg::CarPos> carPos_sub_;
        message_filters::Subscriber<robot_interface::msg::GameInfo> gameInfo_sub_;
        message_filters::Subscriber<robot_interface::msg::Serial> serial_sub_;

        typedef message_filters::sync_policies::ApproximateTime<robot_interface::msg::CarHP, robot_interface::msg::CarPos, robot_interface::msg::GameInfo, robot_interface::msg::Serial> ApproximateSyncPolicy;
        std::unique_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> TS_sync_;

        std::shared_ptr<RobotDecisionSys> myRDS;

        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_action_client_;

        std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
        std::chrono::milliseconds server_timeout_;

        using NavThroughPosesGoalHandle =
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
        NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;
        nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;

        std::shared_ptr<rclcpp::WallTimer<std::_Bind<void (rdsys::RobotDecisionNode::*(rdsys::RobotDecisionNode *))()>, (void *)nullptr>> timer_;

        rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr
            nav_through_poses_feedback_sub_;
        rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::SharedPtr
            nav_through_poses_goal_status_sub_;

    private:
        void makeNewGoal(double x, double y, double &theta);
        std::vector<RobotPosition> point2f2Position(std::array<robot_interface::msg::Point2f, 10UL> pos);
        void messageCallBack(const std::shared_ptr<robot_interface::msg::CarHP const> &carHP_msg_,
                             const std::shared_ptr<robot_interface::msg::CarPos const> &carPos_msg_,
                             const std::shared_ptr<robot_interface::msg::GameInfo const> &gameInfo_msg_,
                             const std::shared_ptr<robot_interface::msg::Serial const> &serial_sub_);
        void nav2FeedBackCallBack(const nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg);
        void nav2GoalStatusCallBack(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
        void respond();

    public:
        RobotDecisionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~RobotDecisionNode();

    public:
        void init(char *waypointsPath, char *decisionsPath);
        bool process_once(int &_HP, int &mode, float &_x, float &_y, int &time, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions);
    };
}
#include "./robot_decision/RobotDecision.h"

#include "robot_interface/msg/car_hp.hpp"
#include "robot_interface/msg/car_pos.hpp"
#include "robot_interface/msg/game_info.hpp"
#include "robot_interface/msg/sentry.hpp"

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
    class RobotDecisionNode : public rclcpp::Node
    {
    private:
        message_filters::Subscriber<robot_interface::msg::CarHP> carHP_sub_;
        message_filters::Subscriber<robot_interface::msg::CarPos> carPos_sub_;
        message_filters::Subscriber<robot_interface::msg::GameInfo> gameInfo_sub_;
        message_filters::Subscriber<robot_interface::msg::Sentry> sentry_sub_;

        typedef message_filters::sync_policies::ApproximateTime<robot_interface::msg::CarHP, robot_interface::msg::CarPos, robot_interface::msg::GameInfo, robot_interface::msg::Sentry> ApproximateSyncPolicy;
        message_filters::Synchronizer<ApproximateSyncPolicy> TS_sync_;

        std::unique_ptr<RobotDecisionSys> myRDS;

        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client;

        std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;

    private:
        void makeNewGoal(double x, double y, double theta);

        void messageCallBack(const robot_interface::msg::CarHP &carHP_msg_, const robot_interface::msg::CarPos &carPos_msg_, const robot_interface::msg::GameInfo gameInfo_msg_, const robot_interface::msg::Sentry &sentry_msg_);

    public:
        RobotDecisionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~RobotDecisionNode();

    public:
        void init(char *waypointsPath, char *decisionsPath);
        void process_once(int _HP, int mode, float _x, float _y, int time, std::vector<RobotPosition> friendPositions, std::vector<RobotPosition> enemyPositions);
    };
}
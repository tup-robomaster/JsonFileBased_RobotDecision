#include "./robot_decision/RobotDecision.h"

#include "global_interface/msg/obj_hp.hpp"
#include "global_interface/msg/car_pos.hpp"
#include "global_interface/msg/game_info.hpp"
#include "global_interface/msg/serial.hpp"
#include "global_interface/msg/detection_array.hpp"
#include "global_interface/msg/decision.hpp"
#include "global_interface/msg/autoaim.hpp"
#include "global_interface/msg/mode_set.hpp"
#include "global_interface/msg/strike_licensing.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_behavior_tree/plugins/action/navigate_through_poses_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <message_filters/subscriber.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace rdsys
{
    using namespace std::chrono_literals;

    class RobotDecisionNode : public rclcpp::Node
    {
    public:
        bool _Debug = false;
        bool _auto_mode = true;

        double delta_yaw = 0.;

    private:
        std::string _WayPointsPath;
        std::string _DecisionsPath;
        float _INIT_DISTANCE_THR;
        float _INIT_SEEK_THR;
        bool _INIT_IsBlue;
        bool _INIT_IFSHOWUI;
        bool _INIT_IFUSEMANUAL;
        int _INIT_SELFINDEX;
        int _INIT_FRIENDOUTPOSTINDEX;
        int _INIT_FRIENDBASEINDEX;
        int _GAME_TIME;
        int _TIME_THR;

        std::string _MAP_PATH;
        float _REAL_WIDTH;
        float _REAL_HEIGHT;
        float _STEP_DISTANCE;
        float _CAR_SEEK_FOV;

    private:
        // RealParamValues:
        int _selfIndex = 5;
        int _friendOutPostIndex = 6;
        int _friendBaseIndex = 7;
        bool _IfShowUI = false;
        bool _IsBlue = false;

        // TempParams:
        float _distance_THR_Temp = 0.5;
        float _seek_THR_Temp = 5.0;
        bool _IfShowUI_Temp = false;
        bool _IsBlue_Temp = false;

        const std::map<std::string, int> type_id = {std::map<std::string, int>::value_type("R1", 0),
                                                    std::map<std::string, int>::value_type("R2", 1),
                                                    std::map<std::string, int>::value_type("R3", 2),
                                                    std::map<std::string, int>::value_type("R4", 3),
                                                    std::map<std::string, int>::value_type("R5", 4),
                                                    std::map<std::string, int>::value_type("R7", 5),
                                                    std::map<std::string, int>::value_type("B1", 6),
                                                    std::map<std::string, int>::value_type("B2", 7),
                                                    std::map<std::string, int>::value_type("B3", 8),
                                                    std::map<std::string, int>::value_type("B4", 9),
                                                    std::map<std::string, int>::value_type("B5", 10),
                                                    std::map<std::string, int>::value_type("B7", 11)};

        const std::vector<int> _car_ids = {0, 1, 2, 3, 4, 6};

    private:
        message_filters::Subscriber<global_interface::msg::ObjHP> objHP_sub_;
        message_filters::Subscriber<global_interface::msg::CarPos> carPos_sub_;
        message_filters::Subscriber<global_interface::msg::GameInfo> gameInfo_sub_;
        message_filters::Subscriber<global_interface::msg::Serial> serial_sub_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<global_interface::msg::Autoaim>::SharedPtr autoaim_sub_;
        rclcpp::Subscription<global_interface::msg::DetectionArray>::SharedPtr detectionArray_sub_;
        rclcpp::Subscription<global_interface::msg::ModeSet>::SharedPtr modeSet_sub_;

        typedef message_filters::sync_policies::ApproximateTime<global_interface::msg::ObjHP,
                                                                global_interface::msg::CarPos,
                                                                global_interface::msg::GameInfo,
                                                                global_interface::msg::Serial>
            ApproximateSyncPolicy;
        std::unique_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> TS_sync_;

        std::shared_ptr<RobotDecisionSys> myRDS;

        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_action_client_;

        std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
        std::chrono::milliseconds server_timeout_;

        nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;

        std::shared_ptr<rclcpp::WallTimer<std::_Bind<void (rdsys::RobotDecisionNode::*(rdsys::RobotDecisionNode *))()>, (void *)nullptr>> timer_;

        rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr
            nav_through_poses_feedback_sub_;
        rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::SharedPtr
            nav_through_poses_goal_status_sub_;

        rclcpp::Publisher<global_interface::msg::Decision>::SharedPtr
            decision_pub_;
        rclcpp::Publisher<global_interface::msg::StrikeLicensing>::SharedPtr
            strikeLicensing_pub_;

        std::shared_timed_mutex myMutex_status;
        std::shared_timed_mutex myMutex_NTP_FeedBack;
        std::shared_timed_mutex myMutex_joint_states;
        std::shared_timed_mutex myMutex_detectionArray;
        std::shared_timed_mutex myMutex_modeSet;
        std::shared_timed_mutex myMutex_autoaim;

        int8_t goal_status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        nav2_msgs::action::NavigateThroughPoses_FeedbackMessage::SharedPtr current_NTP_FeedBack_msg = nullptr;

        sensor_msgs::msg::JointState::SharedPtr joint_states_msg = nullptr;
        global_interface::msg::Autoaim::SharedPtr autoaim_msg = nullptr;
        global_interface::msg::DetectionArray::SharedPtr detectionArray_msg = nullptr;
        global_interface::msg::ModeSet::SharedPtr modeSet_msg = nullptr;

        std::shared_ptr<Decision> excuting_decision = nullptr;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::shared_ptr<geometry_msgs::msg::TransformStamped> _transformStamped = nullptr;

    private:
        /**
         * @brief 向目标缓冲区添加目标点
         * @param x
         * 目标x坐标
         * @param y
         * 目标y坐标
         * @param theta
         * 目标theta角
         */
        void makeNewGoal(double x, double y, double theta);
        /**
         * @brief 将消息位置转换成RobotPosition结构体
         * @param pos
         * 位置集合
         * @return
         * RobotPosition集合
         */
        std::vector<RobotPosition> point2f2Position(std::array<global_interface::msg::Point2f, 12UL> pos);
        /**
         * @brief 消息回调，由message_filters处理
         */
        void messageCallBack(const std::shared_ptr<global_interface::msg::ObjHP const> &carHP_msg_,
                             const std::shared_ptr<global_interface::msg::CarPos const> &carPos_msg_,
                             const std::shared_ptr<global_interface::msg::GameInfo const> &gameInfo_msg_,
                             const std::shared_ptr<global_interface::msg::Serial const> &serial_sub_);
        /**
         * @brief joint_states消息回调
         */
        void jointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr msg);
        /**
         * @brief armor_info消息回调
         */
        void autoaimCallBack(const global_interface::msg::Autoaim::SharedPtr msg);
        /**
         * @brief 检测目标消息回调
         */
        void detectionArrayCallBack(const global_interface::msg::DetectionArray::SharedPtr msg);
        /**
         * @brief 云台手模式设置消息回调
         */
        void modeSetCallBack(const global_interface::msg::ModeSet::SharedPtr msg);
        /**
         * @brief nav2反馈回调
         */
        void nav2FeedBackCallBack(const nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg);
        /**
         * @brief nav2目标状态回调
         */
        void nav2GoalStatusCallBack(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
        /**
         * @brief 周期性参数回调
         */
        void respond();
        /**
         * @brief 决策处理一次
         * @param _HP
         * 当前血量
         * @param mode
         * 当前模式
         * @param _x
         * 当前x轴坐标
         * @param _y
         * 当前y轴坐标
         * @param time
         * 比赛时间
         * @param now_out_post_HP
         * 友方前哨站当前血量
         * @param friendPositions
         * 友方位置集合
         * @param enemyPositions
         * 敌方位置集合
         * @return
         * 处理是否成功
         */
        bool process_once(int &_HP, int &mode, float &_x, float &_y, int &time, int &now_out_post_HP, int &now_base_HP, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, geometry_msgs::msg::TransformStamped::SharedPtr transformStamped);
        /**
         * @brief 根据决策创建消息
         * @param decision
         * 决策
         * @param theta
         * 车辆朝向（弧度制）（偏角）
         * @return
         * 决策消息
         */
        global_interface::msg::Decision makeDecisionMsg(std::shared_ptr<Decision> decision, double &theta);
        /**
         * @brief 手动创建决策消息
         * @param mode
         * 决策模式
         * @param theta
         * 车辆朝向（弧度制）（偏角）
         * @param _x
         * 决策x坐标
         * @param _y
         * 决策y坐标
         * @return
         * 决策消息
         */
        global_interface::msg::Decision makeDecisionMsg(int mode, double theta, float _x, float _y);
        /**
         * @brief 载入配置信息
         * @return
         * 是否成功
         */
        bool decodeConfig();
        /**
         * @brief 清空目标
         */
        void clearGoals();

    public:
        RobotDecisionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~RobotDecisionNode();

    public:
        /**
         * @brief 初始化
         * @param waypointsPath
         * 路径点Json文件路径
         * @param decisionsPath
         * 决策Json文件路径
         */
        void init();
    };
}
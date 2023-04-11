#include "../include/robot_decision_node.hpp"

using namespace std::placeholders;

namespace rdsys
{
    RobotDecisionNode::RobotDecisionNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("robot_decision_node", options)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "RobotDecision node...");
        RCLCPP_INFO(
            this->get_logger(),
            "starting...");
        this->init(WayPointsPATH, DecisionsPATH);
    }

    RobotDecisionNode::~RobotDecisionNode()
    {
    }

    void RobotDecisionNode::init(char *waypointsPath, char *decisionsPath)
    {
        this->declare_parameter<float>("distance_thr", INIT_DISTANCE_THR);
        this->declare_parameter<float>("seek_thr", INIT_SEEK_THR);
        this->declare_parameter<bool>("IsRed", INIT_ISRED);
        this->declare_parameter<bool>("IfShowUI", INIT_IFSHOWUI);
        this->declare_parameter<int>("SelfIndex", INIT_SELFINDEX);
        this->declare_parameter<int>("friendOutPostIndex", INIT_FRIENDOUTPOSTINDEX);

        this->myRDS = std::make_shared<RobotDecisionSys>(RobotDecisionSys(this->_distance_THR_Temp, this->_seek_THR_Temp));

        this->timer_ = this->create_wall_timer(1000ms, std::bind(&RobotDecisionNode::respond, this));

        if (!this->myRDS->decodeWayPoints(waypointsPath))
            RCLCPP_ERROR(
                this->get_logger(),
                "Decode waypoints failed");
        if (!this->myRDS->decodeDecisions(decisionsPath))
            RCLCPP_ERROR(
                this->get_logger(),
                "Decode decisions failed");

        rclcpp::QoS qos(0);
        qos.keep_last(20);
        qos.best_effort();
        qos.durability();
        qos.durability_volatile();

        this->objHP_sub_.subscribe(this, "/obj_hp", qos.get_rmw_qos_profile());
        this->carPos_sub_.subscribe(this, "/car_pos", qos.get_rmw_qos_profile());
        this->gameInfo_sub_.subscribe(this, "/game_info", qos.get_rmw_qos_profile());
        this->serial_sub_.subscribe(this, "/serial_msg", qos.get_rmw_qos_profile());

        this->joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", qos, std::bind(&RobotDecisionNode::jointStateCallBack, this, _1));
        this->detectionArray_sub_ = this->create_subscription<global_interface::msg::DetectionArray>("perception_detector/perception_array", qos, std::bind(&RobotDecisionNode::detectionArrayCallBack, this, _1));

        this->TS_sync_.reset(new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(10), this->objHP_sub_, this->carPos_sub_, this->gameInfo_sub_, this->serial_sub_));
        this->TS_sync_->registerCallback(std::bind(&RobotDecisionNode::messageCallBack, this, _1, _2, _3, _4));

        this->nav_through_poses_feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>(
            "navigate_through_poses/_action/feedback",
            qos,
            std::bind(&RobotDecisionNode::nav2FeedBackCallBack, this, _1));
        this->nav_through_poses_goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "navigate_through_poses/_action/status",
            qos,
            std::bind(&RobotDecisionNode::nav2GoalStatusCallBack, this, _1));

        this->aim_yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("robot_decision/aim_yaw", qos);
        this->decision_pub_ = this->create_publisher<global_interface::msg::Decision>("robot_decision/decision", qos);

        RCLCPP_INFO(
            this->get_logger(),
            "Starting action_client");
        this->nav_through_poses_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");
        if (!nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Action server not available after waiting");
            return;
        }
    }

    bool RobotDecisionNode::process_once(int &_HP, int &mode, float &_x, float &_y, int &time, int &now_out_post_HP, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, geometry_msgs::msg::TransformStamped::SharedPtr transformStamped)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Heartbeat Processing");
        if (_x == 0.0 || _y == 0.0 || std::isnan(_x) || std::isnan(_y))
        {
            if (transformStamped != nullptr)
            {
                _x = -transformStamped->transform.translation.x;
                _y = -transformStamped->transform.translation.y;
            }
            else
            {
                std::shared_lock<std::shared_timed_mutex> slk_1(this->myMutex_NTP_FeedBack);
                if (this->current_NTP_FeedBack_msg != nullptr && this->current_NTP_FeedBack_msg->feedback.current_pose.pose.position.x > 0.0 &&
                    this->current_NTP_FeedBack_msg->feedback.current_pose.pose.position.y > 0.0 &&
                    rclcpp::Clock().now().seconds() - this->current_NTP_FeedBack_msg->feedback.current_pose.header.stamp.sec <= 1)
                {
                    _x = this->current_NTP_FeedBack_msg->feedback.current_pose.pose.position.x;
                    _y = this->current_NTP_FeedBack_msg->feedback.current_pose.pose.position.y;
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Cannot get current Position [Level 1]! Use current Position [Level 2]!");
                }
                else
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Cannot get current Position either [Level 1] & [Level 2]!");
                    slk_1.unlock();
                    return false;
                }
                slk_1.unlock();
            }
        }
        double aim_yaw = this->myRDS->decideAngleByEnemyPos(_x, _y, enemyPositions);
        double roll, pitch, yaw;
        if (aim_yaw != -1)
        {
            tf2::Quaternion nv2_quat;
            if (this->current_NTP_FeedBack_msg != nullptr)
            {
                tf2::fromMsg(this->current_NTP_FeedBack_msg->feedback.current_pose.pose.orientation, nv2_quat);
            }
            else if (transformStamped != nullptr)
            {
                tf2::fromMsg(transformStamped->transform.rotation, nv2_quat);
            }
            else
            {
                nv2_quat.setRPY(0, 0, 0);
            }
            tf2::Matrix3x3 m(nv2_quat);
            m.getRPY(roll, pitch, yaw);
            yaw = -yaw;
            std_msgs::msg::Float32 aim_yaw_msg;
            double delta_yaw = double(aim_yaw - yaw);
            if (delta_yaw > CV_PI)
            {
                delta_yaw = -(CV_PI - (delta_yaw - CV_PI));
            }
            aim_yaw_msg.set__data(delta_yaw);
            RCLCPP_INFO(
                this->get_logger(),
                "Publish Aim Yaw : %lf | aim_yaw : %lf current_yaw: %lf | angle: %lf",
                delta_yaw, aim_yaw, yaw, delta_yaw * 180. / CV_PI);
            this->aim_yaw_pub_->publish(aim_yaw_msg);
        }
        else
        {
            yaw = 0.;
            RCLCPP_WARN(
                this->get_logger(),
                "None Aim Yaw");
        }

        this->acummulated_poses_.clear();
        int myWayPointID = this->myRDS->checkNowWayPoint(_x, _y);
        std::vector<int> availableDecisionID;
        std::map<int, int> id_pos_f, id_pos_e;
        std::shared_ptr<Decision> myDecision = this->myRDS->decide(myWayPointID, mode, _HP, time, now_out_post_HP, friendPositions, enemyPositions, availableDecisionID, id_pos_f, id_pos_e);
        if (myDecision == nullptr)
        {
            return false;
        }
        std::shared_lock<std::shared_timed_mutex> slk_2(this->myMutex_status);
        std::shared_lock<std::shared_timed_mutex> slk_3(this->myMutex_NTP_FeedBack);
        if (this->goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED || this->goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        {
            if (this->excuting_decision != nullptr && this->current_NTP_FeedBack_msg != nullptr && myDecision->weight > this->excuting_decision->weight && this->current_NTP_FeedBack_msg->feedback.estimated_time_remaining.sec > GOAL_TIME_THR_SEC)
            {
                nav_through_poses_action_client_->async_cancel_goals_before(rclcpp::Clock().now());
                this->nav_through_poses_goal_handle_.reset();
                RCLCPP_INFO(
                    this->get_logger(),
                    "Cancel Previous Goal");
            }
            else
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Function Normal Continue");
                global_interface::msg::Decision myDecision_msg;
                myDecision_msg.set__decision_id(myDecision->id);
                myDecision_msg.set__mode(myDecision->decide_mode);
                std::shared_ptr<WayPoint> aimWayPoint = this->myRDS->getWayPointByID(this->excuting_decision->decide_wayPoint);
                myDecision_msg.set__x(aimWayPoint->x);
                myDecision_msg.set__y(aimWayPoint->y);
                RCLCPP_INFO(
                    this->get_logger(),
                    "Publish Previous Decision : [id] %d [mode] %d [x,y] %lf %lf",
                    myDecision_msg.decision_id, myDecision_msg.mode, myDecision_msg.x, myDecision_msg.y);
                this->decision_pub_->publish(myDecision_msg);
                if (this->_IfShowUI)
                {
                    std::vector<std::shared_ptr<WayPoint>> aimWayPoints;
                    if (!this->excuting_decision->if_succession)
                    {
                        aimWayPoints.emplace_back(this->myRDS->getWayPointByID(this->excuting_decision->decide_wayPoint));
                    }
                    else
                    {
                        aimWayPoints = this->myRDS->calculatePath(myWayPointID, this->excuting_decision->decide_wayPoint);
                    }
                    std::shared_lock<std::shared_timed_mutex> slk_4(this->myMutex_joint_states);
                    this->myRDS->UpdateDecisionMap(this->excuting_decision->id, availableDecisionID, myWayPointID, yaw, cv::Point2f(_x, _y), this->joint_states_msg != nullptr ? yaw - this->joint_states_msg->position[0] : -1, aim_yaw, friendPositions, enemyPositions, id_pos_f, id_pos_e, aimWayPoints);
                    slk_4.unlock();
                }
                return true;
            }
        }
        this->excuting_decision = myDecision;
        slk_2.unlock();
        slk_3.unlock();
        std::vector<std::shared_ptr<WayPoint>> aimWayPoints;
        if (!myDecision->if_succession)
        {
            aimWayPoints.emplace_back(this->myRDS->getWayPointByID(myDecision->decide_wayPoint));
        }
        else
        {
            aimWayPoints = this->myRDS->calculatePath(myWayPointID, myDecision->decide_wayPoint);
        }

        if (aimWayPoints.empty())
        {
            return false;
        }
        for (auto &it : aimWayPoints)
        {
            this->makeNewGoal(it->x, it->y, aim_yaw == -1 ? it->theta : aim_yaw);
        }
        this->nav_through_poses_goal_.poses = this->acummulated_poses_;
        RCLCPP_INFO(
            this->get_logger(),
            "Sending a path of %zu waypoints:",
            this->nav_through_poses_goal_.poses.size());
        for (auto waypoint : this->nav_through_poses_goal_.poses)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
        }
        auto send_goal_options =
            rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto)
        {
            this->nav_through_poses_goal_handle_.reset();
        };

        if (this->nav_through_poses_action_client_->wait_for_action_server(std::chrono::microseconds(10)))
        {
            auto future_goal_handle = nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
            // this->nav_through_poses_goal_handle_ = future_goal_handle.get();
            // 问题：future获取时阻塞
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Action server still not available !");
        }
        global_interface::msg::Decision myDecision_msg;
        myDecision_msg.set__decision_id(myDecision->id);
        myDecision_msg.set__mode(myDecision->decide_mode);
        myDecision_msg.set__x(aimWayPoints.back()->x);
        myDecision_msg.set__y(aimWayPoints.back()->y);
        RCLCPP_INFO(
            this->get_logger(),
            "Publish Decision : [id] %d [mode] %d [x,y] %lf %lf",
            myDecision_msg.decision_id, myDecision_msg.mode, myDecision_msg.x, myDecision_msg.y);
        this->decision_pub_->publish(myDecision_msg);
        if (this->_IfShowUI)
        {
            std::shared_lock<std::shared_timed_mutex> slk_4(this->myMutex_joint_states);
            this->myRDS->UpdateDecisionMap(myDecision->id, availableDecisionID, myWayPointID, this->joint_states_msg != nullptr ? this->joint_states_msg->position[0] : -1, cv::Point2f(_x, _y), yaw, aim_yaw, friendPositions, enemyPositions, id_pos_f, id_pos_e, aimWayPoints);
            slk_4.unlock();
        }
        return true;
    }

    void RobotDecisionNode::makeNewGoal(double x, double y, double &theta)
    {
        auto pose = geometry_msgs::msg::PoseStamped();

        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "map_decision";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);

        this->acummulated_poses_.emplace_back(pose);
    }

    std::vector<RobotPosition> RobotDecisionNode::point2f2Position(std::array<global_interface::msg::Point2f, 10UL> pos)
    {
        if (pos.size() != 10)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Position msg not valid !");
            return {};
        }
        std::vector<RobotPosition> result;
        for (int i = 0; i < int(pos.size()); ++i)
        {
            result.emplace_back(RobotPosition(i, pos[i].x, pos[i].y));
        }
        return result;
    }

    void RobotDecisionNode::messageCallBack(const std::shared_ptr<global_interface::msg::ObjHP const> &objHP_msg_,
                                            const std::shared_ptr<global_interface::msg::CarPos const> &objPos_msg_,
                                            const std::shared_ptr<global_interface::msg::GameInfo const> &gameInfo_msg_,
                                            const std::shared_ptr<global_interface::msg::Serial const> &serial_sub_)
    {
        auto start_t = std::chrono::system_clock::now().time_since_epoch();
        geometry_msgs::msg::TransformStamped::SharedPtr transformStamped = nullptr;
        for (int i = 0; i < int(objHP_msg_->hp.size()); ++i)
        {
            if (std::isnan(objHP_msg_->hp[i]))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Receive ObjHP Msg Error");
                return;
            }
            RCLCPP_DEBUG(
                this->get_logger(),
                "Receive ObjHP Msg %d: %d",
                i, objHP_msg_->hp[i]);
        }
        for (int i = 0; i < int(objPos_msg_->pos.size()); ++i)
        {
            if (std::isnan(objPos_msg_->pos[i].x) || std::isnan(objPos_msg_->pos[i].y))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Receive ObjPos Msg Error");
                return;
            }
            RCLCPP_DEBUG(
                this->get_logger(),
                "Receive ObjPos Msg %d: x=%lf, y=%lf",
                i, objPos_msg_->pos[i].x, objPos_msg_->pos[i].y);
        }
        if (std::isnan(gameInfo_msg_->timestamp))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Receive GameInfo Msg Error");
            return;
        }
        RCLCPP_DEBUG(
            this->get_logger(),
            "Receive GameInfo Msg : timestamp=%d",
            gameInfo_msg_->timestamp);
        if (std::isnan(serial_sub_->mode) || std::isnan(serial_sub_->theta))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Receive Serial Msg Error");
            return;
        }
        RCLCPP_DEBUG(
            this->get_logger(),
            "Receive Serial Msg : mode=%d, theta=%lf",
            serial_sub_->mode, serial_sub_->theta);

        this->nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();
        int myHP = objHP_msg_->hp[this->_selfIndex];
        float myPos_x_ = objPos_msg_->pos[this->_selfIndex].x;
        float myPos_y_ = objPos_msg_->pos[this->_selfIndex].y;
        int nowTime = GAME_TIME - gameInfo_msg_->timestamp;
        int mode = serial_sub_->mode;
        int now_out_post_HP = objHP_msg_->hp[this->_friendOutPostIndex];
        std::vector<RobotPosition> allPositions = this->point2f2Position(objPos_msg_->pos);
        try
        {
            transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>(this->tf_buffer_->lookupTransform("map_decision", "gimbal_yaw_frame", tf2::TimePointZero));
            std::shared_lock<std::shared_timed_mutex> slk(this->myMutex_detectionArray);
            if (this->detectionArray_msg != nullptr && rclcpp::Clock().now().seconds() - this->detectionArray_msg->header.stamp.sec <= 5)
            {
                for (auto it : this->detectionArray_msg->detections)
                {
                    tf2::Transform tf2_transform;
                    tf2::Vector3 p(it.center.position.x, it.center.position.y, it.center.position.z);
                    tf2::convert(transformStamped->transform, tf2_transform);
                    p = tf2_transform * p;
                    allPositions[this->type_id.find(it.type)->second].x = p.getX();
                    allPositions[this->type_id.find(it.type)->second].y = p.getY();
                }
            }
            slk.unlock();
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Cannot get transform ! TransformException: %s",
                ex.what());
        }
        std::vector<RobotPosition> friendPositions;
        std::vector<RobotPosition> enemyPositions;
        for (int i = 0; i < int(allPositions.size()); ++i)
        {
            if (i == this->_selfIndex)
            {
                continue;
            }
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
        if (!this->process_once(myHP, mode, myPos_x_, myPos_y_, nowTime, now_out_post_HP, friendPositions, enemyPositions, transformStamped))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Decision failed!");
        }
        auto end_t = std::chrono::system_clock::now().time_since_epoch();
        RCLCPP_DEBUG(
            this->get_logger(),
            "Take Time: %ld nsec FPS: %d",
            end_t.count() - start_t.count(), int(std::chrono::nanoseconds(1000000000).count() / (end_t - start_t).count()));
    }

    void RobotDecisionNode::jointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::unique_lock<std::shared_timed_mutex> ulk(this->myMutex_joint_states);
        if (msg->name[0] == "gimbal_yaw_joint" && !std::isnan(msg->position[0]))
        {
            this->joint_states_msg = msg;
        }
        ulk.unlock();
        RCLCPP_DEBUG(
            this->get_logger(),
            "jointState Recived: yaw = %lf , pitch = %lf",
            msg->position[0], msg->position[1]);
    }

    void RobotDecisionNode::detectionArrayCallBack(const global_interface::msg::DetectionArray::SharedPtr msg)
    {
        std::unique_lock<std::shared_timed_mutex> ulk(this->myMutex_detectionArray);
        this->detectionArray_msg = msg;
        ulk.unlock();
        RCLCPP_DEBUG(
            this->get_logger(),
            "Detection Array Recived: %d || %d",
            msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    void RobotDecisionNode::nav2FeedBackCallBack(const nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg)
    {
        std::unique_lock<std::shared_timed_mutex> ulk(this->myMutex_NTP_FeedBack);
        this->current_NTP_FeedBack_msg = msg;
        ulk.unlock();
        RCLCPP_DEBUG(
            this->get_logger(),
            "Receive Nav2FeedBack: Distance Remainimg: %f Current Pose: x=%lf , y=%lf , z=%lf Time Remaining: %d",
            msg->feedback.distance_remaining, msg->feedback.current_pose.pose.position.x, msg->feedback.current_pose.pose.position.y, msg->feedback.current_pose.pose.position.z, msg->feedback.estimated_time_remaining.sec);
    }

    void RobotDecisionNode::nav2GoalStatusCallBack(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        std::shared_lock<std::shared_timed_mutex> slk(this->myMutex_status);
        bool check = (this->goal_status != msg->status_list.back().status);
        slk.unlock();
        if (check)
        {
            std::unique_lock<std::shared_timed_mutex> ulk(this->myMutex_status);
            this->goal_status = msg->status_list.back().status;
            ulk.unlock();
        }
        if (msg->status_list.back().status != action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Nav2StatusCallBack Status: %d",
                msg->status_list.back().status);
        }
    }

    void RobotDecisionNode::respond()
    {
        this->get_parameter("distance_thr", this->_distance_THR_Temp);
        this->get_parameter("seek_thr", this->_seek_THR_Temp);
        this->get_parameter("IfShowUI", this->_IfShowUI_Temp);
        this->get_parameter("IsRed", this->_IsRed_Temp);
        this->get_parameter("SelfIndex", this->_selfIndex_Temp);
        this->get_parameter("friendOutPostIndex", this->_friendOutPostIndex_Temp);

        if (this->myRDS->getDistanceTHR() != this->_distance_THR_Temp)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "set _distance_THR to %f",
                this->_distance_THR_Temp);
            this->myRDS->setDistanceTHR(this->_distance_THR_Temp);
        }
        if (this->myRDS->getSeekTHR() != this->_seek_THR_Temp)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "set _seek_THR to %f",
                this->_seek_THR_Temp);
            this->myRDS->setSeekTHR(this->_seek_THR_Temp);
        }
        if (this->myRDS->getIfShowUI() != this->_IfShowUI_Temp)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "set _IfShowUI to %d",
                this->_IfShowUI_Temp);
            this->myRDS->setIfShowUI(this->_IfShowUI_Temp);
            this->_IfShowUI = this->_IfShowUI_Temp;
        }
        if (this->_IsRed != this->_IsRed_Temp)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "set _IsRed to %d",
                this->_IsRed_Temp);
            this->_IsRed = this->_IsRed_Temp;
        }
        if (this->_selfIndex != this->_selfIndex_Temp)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "set _selfIndex to %d",
                this->_selfIndex_Temp);
            this->_selfIndex = this->_selfIndex_Temp;
        }
        if (this->_friendOutPostIndex != this->_friendOutPostIndex_Temp)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "set _friendOutPostIndex to %d",
                this->_friendOutPostIndex_Temp);
            this->_friendOutPostIndex = this->_friendOutPostIndex_Temp;
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
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
        if (!this->decodeConfig())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to get Config!");
            abort();
        }
        this->init();
        assert(CARPOS_NUM == this->type_id.size() / 2);
    }

    RobotDecisionNode::~RobotDecisionNode()
    {
    }

    bool RobotDecisionNode::decodeConfig()
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_decision");
        Json::Reader jsonReader;
        Json::Value jsonValue;
        std::ifstream jsonFile(package_share_directory + "/" + "JsonFile/config.json");
        if (!jsonReader.parse(jsonFile, jsonValue, true))
        {
            std::cout << "read error" << std::endl;
            jsonFile.close();
            return false;
        }
        Json::Value arrayValue = jsonValue["config"];
        this->_Debug = arrayValue["Debug"].asBool();
        this->_WayPointsPath = package_share_directory + "/JsonFile/" + arrayValue["WayPointsPATH"].asCString();
        this->_DecisionsPath = package_share_directory + "/JsonFile/" + arrayValue["DecisionsPATH"].asCString();
        this->_INIT_DISTANCE_THR = arrayValue["INIT_DISTANCE_THR"].asFloat();
        this->_INIT_SEEK_THR = arrayValue["INIT_SEEK_THR"].asFloat();
        this->_INIT_IsBlue = arrayValue["INIT_ISBLUE"].asBool();
        this->_INIT_IFSHOWUI = arrayValue["INIT_IFSHOWUI"].asBool();
        this->_INIT_IFUSEMANUAL = arrayValue["INIT_IFUSEMANUAL"].asBool();
        this->_INIT_SELFINDEX = arrayValue["INIT_SELFINDEX"].asInt();
        this->_INIT_FRIENDOUTPOSTINDEX = arrayValue["INIT_FRIENDOUTPOSTINDEX"].asInt();
        this->_INIT_FRIENDBASEINDEX = arrayValue["INIT_FRIENDBASEINDEX"].asInt();
        this->_GAME_TIME = arrayValue["GAME_TIME"].asInt();
        this->_TIME_THR = arrayValue["TIME_THR"].asInt();

        this->_MAP_PATH = package_share_directory + "/resources/" + arrayValue["MAP_PATH"].asCString();
        this->_REAL_WIDTH = arrayValue["REAL_WIDTH"].asFloat();
        this->_REAL_HEIGHT = arrayValue["REAL_HEIGHT"].asFloat();
        this->_STEP_DISTANCE = arrayValue["STEP_DISTANCE"].asFloat();
        this->_CAR_SEEK_FOV = arrayValue["CAR_SEEK_FOV"].asFloat();
        return true;
    }

    void RobotDecisionNode::init()
    {
        this->declare_parameter<float>("distance_thr", this->_INIT_DISTANCE_THR);
        this->declare_parameter<float>("seek_thr", this->_INIT_SEEK_THR);
        this->declare_parameter<bool>("IsBlue", this->_INIT_IsBlue);
        this->declare_parameter<bool>("IfShowUI", this->_INIT_IFSHOWUI);

        this->_selfIndex = this->_INIT_SELFINDEX;
        this->_friendOutPostIndex = this->_INIT_FRIENDOUTPOSTINDEX;
        this->_friendBaseIndex = this->_INIT_FRIENDBASEINDEX;

        this->myRDS = std::make_shared<RobotDecisionSys>(RobotDecisionSys(this->_distance_THR_Temp, this->_seek_THR_Temp, this->_REAL_WIDTH, this->_REAL_HEIGHT, this->_MAP_PATH, this->_STEP_DISTANCE, this->_CAR_SEEK_FOV));

        this->timer_ = this->create_wall_timer(200ms, std::bind(&RobotDecisionNode::respond, this));

        if (!this->myRDS->decodeWayPoints(this->_WayPointsPath))
            RCLCPP_ERROR(
                this->get_logger(),
                "Decode waypoints failed");
        if (!this->myRDS->decodeDecisions(this->_DecisionsPath))
            RCLCPP_ERROR(
                this->get_logger(),
                "Decode decisions failed");

        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.durability();
        qos.durability_volatile();

        this->objHP_sub_.subscribe(this, "/obj_hp", qos.get_rmw_qos_profile());
        this->carPos_sub_.subscribe(this, "/car_pos", qos.get_rmw_qos_profile());
        this->gameInfo_sub_.subscribe(this, "/game_info", qos.get_rmw_qos_profile());
        this->serial_sub_.subscribe(this, "/serial_msg", qos.get_rmw_qos_profile());

        this->joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", qos, std::bind(&RobotDecisionNode::jointStateCallBack, this, _1));
        this->autoaim_sub_ = this->create_subscription<global_interface::msg::Autoaim>("/armor_detector/armor_msg", qos, std::bind(&RobotDecisionNode::autoaimCallBack, this, _1));
        this->detectionArray_sub_ = this->create_subscription<global_interface::msg::DetectionArray>("perception_detector/perception_array", qos, std::bind(&RobotDecisionNode::detectionArrayCallBack, this, _1));
        this->modeSet_sub_ = this->create_subscription<global_interface::msg::ModeSet>("/mode_set", qos, std::bind(&RobotDecisionNode::modeSetCallBack, this, _1));

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

        this->decision_pub_ = this->create_publisher<global_interface::msg::Decision>("robot_decision/decision", qos);
        this->strikeLicensing_pub_ = this->create_publisher<global_interface::msg::StrikeLicensing>("robot_decision/strikeLicensing", qos);
        RCLCPP_INFO(
            this->get_logger(),
            "Starting action_client");
        this->nav_through_poses_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");
        if (!this->nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Action server not available after waiting");
            return;
        }
        else
        {
            this->nav_through_poses_action_client_->async_cancel_all_goals();
        }
    }

    bool RobotDecisionNode::process_once(int &_HP, int &mode, float &_x, float &_y, int &time, int &now_out_post_HP, int &now_base_HP, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, geometry_msgs::msg::TransformStamped::SharedPtr transformStamped)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Heartbeat Processing");
        if (_x == 0.0 || _y == 0.0 || std::isnan(_x) || std::isnan(_y))
        {
            if (transformStamped != nullptr)
            {
                _x = transformStamped->transform.translation.x;
                _y = transformStamped->transform.translation.y;
            }
            else if (this->_transformStamped != nullptr && abs(this->get_clock()->now().seconds() - this->_transformStamped->header.stamp.sec) < this->_TIME_THR)
            {
                _x = this->_transformStamped->transform.translation.x;
                _y = this->_transformStamped->transform.translation.y;
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Cannot get current Position!");
                return false;
            }
        }
        double aim_yaw = this->myRDS->decideAngleByEnemyPos(_x, _y, enemyPositions);
        double roll, pitch, yaw;
        tf2::Quaternion nv2_quat;
        if (transformStamped != nullptr)
        {
            tf2::fromMsg(transformStamped->transform.rotation, nv2_quat);
        }
        else
        {
            nv2_quat.setRPY(0, 0, 0);
        }
        tf2::Matrix3x3 m(nv2_quat);
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(
            this->get_logger(),
            "Current status: x = %lf , y = %lf , yaw = %lf, HP: %d , MODE: %d",
            _x, _y, yaw, _HP, mode);

        if (aim_yaw != -1)
        {
            this->delta_yaw = double(aim_yaw - yaw);
            if (this->delta_yaw > CV_PI)
            {
                this->delta_yaw = -(2 * CV_PI - this->delta_yaw);
            }
            
        }
        else
        {
            this->delta_yaw = 0.;
            RCLCPP_WARN(
                this->get_logger(),
                "None Aim Yaw");
        }
        
        int myWayPointID = this->myRDS->checkNowWayPoint(_x, _y);
        std::vector<int> availableDecisionID;
        std::map<int, int> id_pos_f, id_pos_e;
        std::shared_ptr<Decision> myDecision = this->myRDS->decide(myWayPointID, mode, _HP, time, now_out_post_HP, now_base_HP, friendPositions, enemyPositions, availableDecisionID, id_pos_f, id_pos_e);
        if (myDecision == nullptr)
        {
            return false;
        }
        // RCLCPP_WARN(
        //                 this->get_logger(),
        //                 "Aim Yaw: %lf",delta_yaw);
        // this->delta_yaw = abs(this->delta_yaw) > 1.2217 && myDecision->decide_mode == 6 ? 0. : this->delta_yaw;
        // RCLCPP_WARN(
        //                 this->get_logger(),
        //                 "Aim Yaw AFTER: %lf",delta_yaw);
        bool auto_flag = true;
        std::unique_lock<std::shared_timed_mutex> slk_modeSet(this->myMutex_modeSet);
        if (!this->_auto_mode)
        {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Not in Auto mode, wait.");
            auto_flag = false;
        }
        slk_modeSet.unlock();
        std::shared_lock<std::shared_timed_mutex> slk_2(this->myMutex_status);
        if (!auto_flag)
        {
            this->excuting_decision = myDecision;
            this->myRDS->UpdateDecisionMap(myDecision->id, availableDecisionID, myWayPointID, yaw, cv::Point2f(_x, _y), (this->joint_states_msg != nullptr && !isnan(this->joint_states_msg->position[0])) ? yaw + this->joint_states_msg->position[0] : -1, aim_yaw, friendPositions, enemyPositions, id_pos_f, id_pos_e);
            return true;
        }
        else if (this->goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED || this->goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        {
            if (this->excuting_decision != nullptr && myDecision->weight > this->excuting_decision->weight)
            {
                nav_through_poses_action_client_->async_cancel_all_goals();
                RCLCPP_INFO(
                    this->get_logger(),
                    "Cancel Previous Goals");
            }
            else
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Function Normal Continue");
                if (this->excuting_decision == nullptr)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to get Previous Decision. Try to clean up!");
                    nav_through_poses_action_client_->async_cancel_all_goals();
                    return false;
                }
                auto myDecision_msg = this->makeDecisionMsg(this->excuting_decision, this->delta_yaw);
                this->decision_pub_->publish(myDecision_msg);
                if (this->_IfShowUI)
                {
                    std::shared_lock<std::shared_timed_mutex> slk_3(this->myMutex_joint_states);
                    this->myRDS->UpdateDecisionMap(this->excuting_decision->id, availableDecisionID, myWayPointID, yaw, cv::Point2f(_x, _y), (this->joint_states_msg != nullptr && !isnan(this->joint_states_msg->position[0])) ? yaw + this->joint_states_msg->position[0] : -1, aim_yaw, friendPositions, enemyPositions, id_pos_f, id_pos_e);
                    slk_3.unlock();
                }
                return true;
            }
        }
        this->excuting_decision = myDecision;
        slk_2.unlock();
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
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to calculatePath");
            return false;
        }
        this->clearGoals();
        for (auto &it : aimWayPoints)
        {
            double temp_waypoint_yaw = myDecision->if_reverse ? it->theta + CV_PI : it->theta;
            this->makeNewGoal(it->x, it->y, aim_yaw == -1 ? temp_waypoint_yaw : aim_yaw);
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

        if (this->nav_through_poses_action_client_->wait_for_action_server(std::chrono::microseconds(100)))
        {
            auto future_goal_handle = nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Action server still not available !");
        }
        global_interface::msg::Decision myDecision_msg = this->makeDecisionMsg(myDecision, this->delta_yaw);
        this->decision_pub_->publish(myDecision_msg);
        if (this->_IfShowUI)
        {
            std::shared_lock<std::shared_timed_mutex> slk_3(this->myMutex_joint_states);
            this->myRDS->UpdateDecisionMap(myDecision->id, availableDecisionID, myWayPointID, yaw, cv::Point2f(_x, _y), (this->joint_states_msg != nullptr && !isnan(this->joint_states_msg->position[0])) ? yaw + this->joint_states_msg->position[0] : -1, aim_yaw, friendPositions, enemyPositions, id_pos_f, id_pos_e);
            slk_3.unlock();
        }
        return true;
    }

    void RobotDecisionNode::makeNewGoal(double x, double y, double theta)
    {
        auto pose = geometry_msgs::msg::PoseStamped();

        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map_decision";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);

        this->acummulated_poses_.emplace_back(pose);
    }

    std::vector<RobotPosition> RobotDecisionNode::point2f2Position(std::array<global_interface::msg::Point2f, 12UL> pos)
    {
        if (pos.size() != CARPOS_NUM * 2)
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
                                            const std::shared_ptr<global_interface::msg::Serial const> &serial_msg_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "----------------------------------------------------------------------------------------[ONCE PROCESS]");
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
        if (std::isnan(gameInfo_msg_->timestamp) || std::isnan(gameInfo_msg_->game_stage))
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
        if (std::isnan(serial_msg_->mode) || std::isnan(serial_msg_->theta))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Receive Serial Msg Error");
            return;
        }
        RCLCPP_DEBUG(
            this->get_logger(),
            "Receive Serial Msg : mode=%d, theta=%lf",
            serial_msg_->mode, serial_msg_->theta);
        int currentSelfIndex = this->_selfIndex;
        int currentFriendOutpostIndex_hp = this->_friendOutPostIndex;
        int currentEnemyOutpostIndex_hp = this->_friendOutPostIndex + OBJHP_NUM;
        int currentBaseIndex_hp = this->_friendBaseIndex;
        int currentSelfIndex_hp = this->_selfIndex;
        if (this->_IsBlue)
        {
            currentSelfIndex_hp = this->_selfIndex + OBJHP_NUM;
            currentFriendOutpostIndex_hp = this->_friendOutPostIndex + OBJHP_NUM;
            currentEnemyOutpostIndex_hp = this->_friendOutPostIndex;
            currentBaseIndex_hp = this->_friendBaseIndex + OBJHP_NUM;
            currentSelfIndex = this->_selfIndex + CARPOS_NUM;
        }
        if (gameInfo_msg_->game_stage != GameStage::IN_BATTLE && !this->_Debug)
        {
            RCLCPP_WARN_ONCE(
                this->get_logger(),
                "Wait for game start ...");
            return;
        }
        this->nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();
        int myHP = objHP_msg_->hp[currentSelfIndex_hp];
        float myPos_x_ = objPos_msg_->pos[currentSelfIndex].x;
        float myPos_y_ = objPos_msg_->pos[currentSelfIndex].y;
        int nowTime = this->_GAME_TIME - gameInfo_msg_->timestamp;
        int mode = serial_msg_->mode;
        int now_out_post_HP = objHP_msg_->hp[currentFriendOutpostIndex_hp];
        int now_out_post_HP_enemy = objHP_msg_->hp[currentEnemyOutpostIndex_hp];
        bool _if_enemy_outpost_down = now_out_post_HP_enemy == 0;
        int now_base_hp = objHP_msg_->hp[currentBaseIndex_hp];
        std::vector<int> _car_hps;
        for (auto &it : this->_car_ids)
        {
            _car_hps.emplace_back(objHP_msg_->hp[it + !this->_IsBlue * OBJHP_NUM]);
        }
        std::vector<RobotPosition> allPositions = this->point2f2Position(objPos_msg_->pos);
        try
        {
            transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>(this->tf_buffer_->lookupTransform("map_decision", "base_link", tf2::TimePointZero));
            this->_transformStamped = transformStamped;
            std::shared_lock<std::shared_timed_mutex> slk(this->myMutex_detectionArray);
            if (this->detectionArray_msg != nullptr && this->get_clock()->now().seconds() - this->detectionArray_msg->header.stamp.sec <= this->_TIME_THR)
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
            if (i == currentSelfIndex)
            {
                continue;
            }
            if (i < CARPOS_NUM)
            {
                if (this->_IsBlue)
                    enemyPositions.emplace_back(allPositions[i]);
                else
                    friendPositions.emplace_back(allPositions[i]);
            }
            else
            {
                if (this->_IsBlue)
                    friendPositions.emplace_back(allPositions[i]);
                else
                    enemyPositions.emplace_back(allPositions[i]);
            }
        }
        RobotPosition myPos;
        myPos.robot_id = this->_IsBlue ? this->type_id.find("B7")->second : this->type_id.find("R7")->second;
        myPos.x = myPos_x_;
        myPos.y = myPos_y_;
        std::map<int, float> target_weights = this->myRDS->decideAimTarget(_IsBlue, myPos, enemyPositions, _car_hps, 0.5, 0.5, _if_enemy_outpost_down);
        RCLCPP_INFO(
            this->get_logger(),
            "Publish StrikeLicensing: ");
        for (auto iter_target_weights = target_weights.begin(); iter_target_weights != target_weights.end(); iter_target_weights++)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "       (id=%d, %lf)",
                iter_target_weights->first, iter_target_weights->second);
        }
        global_interface::msg::StrikeLicensing strikeLicensing_msg;
        strikeLicensing_msg.header.stamp = this->get_clock()->now();
        strikeLicensing_msg.header.frame_id = "decision";
        int ids = 0;
        for (auto iter_target_weights = target_weights.begin(); iter_target_weights != target_weights.end(); iter_target_weights++)
        {
            strikeLicensing_msg.weights[ids] = iter_target_weights->second;
            ++ids;
        }
        this->strikeLicensing_pub_->publish(strikeLicensing_msg);
        if (!this->process_once(myHP, mode, myPos_x_, myPos_y_, nowTime, now_out_post_HP, now_base_hp, friendPositions, enemyPositions, transformStamped))
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

    void RobotDecisionNode::autoaimCallBack(const global_interface::msg::Autoaim::SharedPtr msg)
    {
        std::unique_lock<std::shared_timed_mutex> ulk(this->myMutex_autoaim);
        this->autoaim_msg = msg;
        ulk.unlock();
        RCLCPP_DEBUG(
            this->get_logger(),
            "autoaim Recived");
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

    void RobotDecisionNode::modeSetCallBack(const global_interface::msg::ModeSet::SharedPtr msg)
    {
        if(!_INIT_IFUSEMANUAL)
            return;
        int mode = msg->mode;
        float _x = msg->x;
        float _y = msg->y;
        RCLCPP_INFO(
            this->get_logger(),
            "Manual mode: %d x:%lf y:%lf",
            mode, _x, _y);
        if (mode == 0)
            return;
        bool check = true;
        std::unique_lock<std::shared_timed_mutex> ulk(this->myMutex_modeSet);
        if (this->modeSet_msg == nullptr)
        {
            this->modeSet_msg = msg;
            check = true;
        }
        else if (this->modeSet_msg != nullptr && this->modeSet_msg->mode == mode && this->modeSet_msg->x == _x && this->modeSet_msg->y == _y)
            check = false;
            // return;
        else
        {
            this->modeSet_msg = msg;
            check = true;
        }
        std::shared_lock<std::shared_timed_mutex> slk(this->myMutex_autoaim);
        slk.unlock();
        global_interface::msg::Decision newDecision_msg;
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions send_goal_options;
        switch (mode)
        {
        case 1:
            this->nav_through_poses_action_client_->async_cancel_all_goals();
            this->_auto_mode = true;
            break;
        case 2:
            this->nav_through_poses_action_client_->async_cancel_all_goals();
            this->_auto_mode = false;
            newDecision_msg = this->makeDecisionMsg(Mode::MANUAL_ATTACK, this->delta_yaw, msg->x, msg->y);
            slk.lock();
            if (this->autoaim_msg != nullptr && abs(rclcpp::Clock().now().seconds() - this->autoaim_msg->header.stamp.sec) < this->_TIME_THR && !this->autoaim_msg->is_target_lost)
            {
                newDecision_msg.set__mode(Mode::AUTOAIM);
            }
            else
            {
                newDecision_msg.set__mode(Mode::MANUAL_ATTACK);
            }
            slk.unlock();
            this->decision_pub_->publish(newDecision_msg);
            if (check)
            {
                this->clearGoals();
                this->makeNewGoal(_x, _y, 0);
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
                send_goal_options =
                    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
                // send_goal_options.result_callback = [this, msg](auto)
                // {
                //     global_interface::msg::Decision newDecision_msg = this->makeDecisionMsg(Mode::AUTOAIM, 0, msg->x, msg->y);
                //     this->decision_pub_->publish(newDecision_msg);
                // };
                if (this->nav_through_poses_action_client_->wait_for_action_server(std::chrono::microseconds(100)))
                {
                    auto future_goal_handle = nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
                }
                else
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Action server still not available !");
                }
            }

            break;
        case 3:
            this->nav_through_poses_action_client_->async_cancel_all_goals();
            this->_auto_mode = false;
            newDecision_msg = this->makeDecisionMsg(Mode::MANUAL_BACKDEFENSE, 0., msg->x, msg->y);
            this->decision_pub_->publish(newDecision_msg);
            if (check)
            {
                this->clearGoals();
                this->makeNewGoal(_x, _y, 0);
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
                send_goal_options =
                    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
                send_goal_options.result_callback = [this, msg](auto)
                {
                    global_interface::msg::Decision newDecision_msg = this->makeDecisionMsg(Mode::AUTOAIM, this->delta_yaw, msg->x, msg->y);
                    this->decision_pub_->publish(newDecision_msg);
                };
                if (this->nav_through_poses_action_client_->wait_for_action_server(std::chrono::microseconds(100)))
                {
                    auto future_goal_handle = nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
                }
                else
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Action server still not available !");
                }
            }
            break;
        case 4:
            this->nav_through_poses_action_client_->async_cancel_all_goals();
            this->_auto_mode = false;
            /* code */
            break;

        default:
            break;
        }
        ulk.unlock();
    }

    void RobotDecisionNode::nav2FeedBackCallBack(const nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg)
    {
        std::unique_lock<std::shared_timed_mutex> ulk(this->myMutex_NTP_FeedBack);
        this->current_NTP_FeedBack_msg = msg;
        ulk.unlock();
        RCLCPP_DEBUG(
            this->get_logger(),
            "Receive Nav2FeedBack: Distance Remainimg: %f Current Pose: x=%lf , y=%lf , z=%lf Time Remaining: %d in frame %s",
            msg->feedback.distance_remaining, msg->feedback.current_pose.pose.position.x, msg->feedback.current_pose.pose.position.y, msg->feedback.current_pose.pose.position.z, msg->feedback.estimated_time_remaining.sec, msg->feedback.current_pose.header.frame_id.c_str());
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
        this->get_parameter("IsBlue", this->_IsBlue_Temp);

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
        if (this->_IsBlue != this->_IsBlue_Temp)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "set _IsBlue to %d",
                this->_IsBlue_Temp);
            this->_IsBlue = this->_IsBlue_Temp;
        }
    }

    global_interface::msg::Decision RobotDecisionNode::makeDecisionMsg(std::shared_ptr<Decision> decision, double &theta)
    {
        global_interface::msg::Decision myDecision_msg;
        myDecision_msg.header.frame_id = "base_link";
        myDecision_msg.header.stamp = this->get_clock()->now();
        myDecision_msg.set__decision_id(decision->id);
        std::shared_lock<std::shared_timed_mutex> slk(this->myMutex_autoaim);
        if (this->autoaim_msg != nullptr && abs(rclcpp::Clock().now().seconds() - this->autoaim_msg->header.stamp.sec) < this->_TIME_THR && decision->decide_mode != Mode::AUTOAIM && !this->autoaim_msg->is_target_lost)
        {
            myDecision_msg.set__mode(Mode::AUTOAIM);
        }
        else
        {
            myDecision_msg.set__mode(decision->decide_mode);
        }
        slk.unlock();
        std::shared_ptr<WayPoint> aimWayPoint = this->myRDS->getWayPointByID(decision->decide_wayPoint);
        myDecision_msg.set__x(aimWayPoint->x);
        myDecision_msg.set__y(aimWayPoint->y);
        myDecision_msg.set__theta(theta);
        RCLCPP_INFO(
            this->get_logger(),
            "Publish Decision : [id] %d [mode] %d [x,y] %lf %lf",
            myDecision_msg.decision_id, myDecision_msg.mode, myDecision_msg.x, myDecision_msg.y);
        RCLCPP_INFO(
            this->get_logger(),
            "Publish Aim Yaw : %lf | angle: %lf",
            theta, theta * 180. / CV_PI);
        return myDecision_msg;
    }

    global_interface::msg::Decision RobotDecisionNode::makeDecisionMsg(int mode, double theta, float _x, float _y)
    {
        global_interface::msg::Decision myDecision_msg;
        myDecision_msg.header.frame_id = "base_link";
        myDecision_msg.header.stamp = this->get_clock()->now();
        myDecision_msg.set__decision_id(-1);
        myDecision_msg.set__mode(mode);
        myDecision_msg.set__x(_x);
        myDecision_msg.set__y(_y);
        myDecision_msg.set__theta(theta);
        RCLCPP_INFO(
            this->get_logger(),
            "Publish Decision : [id] %d [mode] %d [x,y] %lf %lf",
            myDecision_msg.decision_id, myDecision_msg.mode, myDecision_msg.x, myDecision_msg.y);
        RCLCPP_INFO(
            this->get_logger(),
            "Publish Aim Yaw : %lf | angle: %lf",
            theta, theta * 180. / CV_PI);
        return myDecision_msg;
    }

    void RobotDecisionNode::clearGoals()
    {
        this->acummulated_poses_.clear();
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
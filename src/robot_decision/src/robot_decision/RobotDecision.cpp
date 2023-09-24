#include "../../include/robot_decision/RobotDecision.h"

namespace rdsys
{
    GameHandler::GameHandler()
    {
    }

    GameHandler::~GameHandler()
    {
    }

    void GameHandler::update(int &gameTime)
    {
        this->lastUpdateTime = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
        this->gameTime = gameTime;
    }

    int RobotDecisionSys::calculatePosition(RobotPosition &pos)
    {
        double distance = FLT_MAX;
        int id = -1;
        for (auto it : this->wayPointMap)
        {
            double tempDistance = sqrtf(powf(double(it->x - pos.x), 2) + powf(double(it->y - pos.y), 2));
            if (tempDistance - distance < 0.)
            {
                distance = tempDistance;
                id = it->id;
            }
        }
        return distance - this->_distance_THR > 0. ? -1 : id;
    }

    std::vector<std::shared_ptr<WayPoint>> RobotDecisionSys::calculatePath(int startWapPointID, int endWapPointID)
    {
        std::vector<std::shared_ptr<WayPoint>> result;
        std::vector<int> container = {startWapPointID};
        std::map<int, bool> waypoint_flag;
        waypoint_flag.insert(std::make_pair(startWapPointID, true));
        bool flag = true;
        while (flag)
        {
            int current_waypoint = container.back();
            auto iter = connection_map.find(current_waypoint);
            bool flag2 = false;
            for (auto &it : iter->second)
            {
                auto iter2 = waypoint_flag.find(it);
                if (iter2 == waypoint_flag.end())
                {
                    waypoint_flag.insert(std::make_pair(it, true));
                    container.emplace_back(it);
                    flag2 = true;
                    flag = it != endWapPointID;
                }
                if (flag2)
                    break;
            }
            if (!flag2)
            {
                container.pop_back();
            }
            if (waypoint_flag.size() == wayPointMap.size())
                break;
        }
        for (auto &it : container)
        {
            if (it != container.front())
                result.emplace_back(this->getWayPointByID(it));
        }
        return result;
    }

    cv::Point2i RobotDecisionSys::createEndPointByTheta(double x1, double y1, double theta, int length)
    {
        theta = theta < 0. ? 2. * CV_PI - abs(theta) : theta;
        return cv::Point2i((int)round(x1 + length * cos(theta)), (int)round(y1 + length * sin(theta)));
    }

    cv::Point2i RobotDecisionSys::createEndPointByTheta(cv::Point start, double theta, int length)
    {
        theta = theta < 0. ? 2. * CV_PI - abs(theta) : theta;
        return cv::Point2i((int)round(start.x + length * cos(theta)), (int)round(start.y + length * sin(theta)));
    }

    bool RobotDecisionSys::checkBlock(cv::Point start, double theta, int distance)
    {
        int temp_step = int(this->_STEP_DISTANCE * (1080. / this->_REAL_HEIGHT));
        for (float distance_step = temp_step; distance_step < distance; distance_step += temp_step)
        {
            cv::Point2i check_point = this->createEndPointByTheta(start, theta, distance_step);
            if ((int)this->decisionMap_Gray.at<uchar>(check_point.y, check_point.x) < 10)
            {
                return true;
            }
        }
        return false;
    }

    RobotDecisionSys::RobotDecisionSys(float &_distance_THR, float &_seek_THR, float &_REAL_WIDTH, float &_REAL_HEIGHT, std::string &_MAP_PATH, float &_STEP_DISTANCE, float &_CAR_SEEK_FOV)
    {
        this->_distance_THR = _distance_THR;
        this->_seek_THR = _seek_THR;
        this->_REAL_WIDTH = _REAL_WIDTH;
        this->_REAL_HEIGHT = _REAL_HEIGHT;
        this->_MAP_PATH = _MAP_PATH;
        this->_STEP_DISTANCE = _STEP_DISTANCE;
        this->_CAR_SEEK_FOV = _CAR_SEEK_FOV;

        this->myGameHandler = std::make_shared<GameHandler>(GameHandler());
        this->decisionMap = cv::imread(this->_MAP_PATH);
        cv::resize(this->decisionMap, this->decisionMap, cv::Size(int(this->_REAL_WIDTH / this->_REAL_HEIGHT * 1080.), 1080));
        cv::cvtColor(this->decisionMap, this->decisionMap_Gray, cv::COLOR_BGR2GRAY, 1);
    }

    RobotDecisionSys::~RobotDecisionSys()
    {
    }

    bool RobotDecisionSys::decodeWayPoints(std::string &filePath)
    {
        Json::Reader jsonReader;
        Json::Value jsonValue;
        std::ifstream jsonFile(filePath);
        if (!jsonReader.parse(jsonFile, jsonValue, true))
        {
            std::cout << "read error" << std::endl;
            jsonFile.close();
            return false;
        }
        Json::Value arrayValue = jsonValue["data"];
        std::vector<WayPoint> tempwayPointMap;
        for (int i = 0; i < int(arrayValue.size()); ++i)
        {
            std::shared_ptr<WayPoint> wayPoint = std::make_shared<WayPoint>(WayPoint());
            wayPoint->id = arrayValue[i]["id"].asInt();
            wayPoint->type = arrayValue[i]["type"].asInt();
            wayPoint->x = arrayValue[i]["x"].asFloat();
            wayPoint->y = arrayValue[i]["y"].asFloat();
            wayPoint->theta = arrayValue[i]["angle"].asDouble();
            Json::Value connectedPoints = arrayValue[i]["connect"];
            for (int j = 0; j < int(connectedPoints.size()); ++j)
            {
                wayPoint->connection.emplace_back(connectedPoints[j].asInt());
            }
            Json::Value enemyWeightsArray = arrayValue[i]["enemyWeights"];
            for (int j = 0; j < int(enemyWeightsArray.size()); ++j)
            {
                wayPoint->enemyWeights[j] = enemyWeightsArray[j].asInt();
            }
            this->wayPointMap.emplace_back(wayPoint);
            this->connection_map.insert(std::make_pair(wayPoint->id, wayPoint->connection));
        }
        jsonFile.close();
        return true;
    }

    bool RobotDecisionSys::decodeDecisions(std::string &filePath)
    {
        Json::Reader jsonReader;
        Json::Value jsonValue;
        std::ifstream jsonFile(filePath);
        if (!jsonReader.parse(jsonFile, jsonValue, true))
        {
            std::cout << "read error" << std::endl;
            jsonFile.close();
            return false;
        }
        Json::Value arrayValue = jsonValue["data"];
        for (int i = 0; i < int(arrayValue.size()); ++i)
        {
            std::shared_ptr<Decision> decision = std::make_shared<Decision>(Decision());
            decision->id = arrayValue[i]["id"].asInt();
            decision->name = arrayValue[i]["name"].asCString();
            Json::Value wayPointIDArray = arrayValue[i]["wayPointID"];
            for (int j = 0; j < int(wayPointIDArray.size()); ++j)
            {
                decision->wayPointID.emplace_back(wayPointIDArray[j].asInt());
            }
            decision->weight = arrayValue[i]["weight"].asInt();
            decision->start_time = arrayValue[i]["start_time"].asInt();
            decision->end_time = arrayValue[i]["end_time"].asInt();
            decision->robot_mode = arrayValue[i]["robot_mode"].asInt();
            decision->_minHP = arrayValue[i]["minHP"].asInt();
            decision->_maxHP = arrayValue[i]["maxHP"].asInt();
            decision->decide_mode = arrayValue[i]["decide_mode"].asInt();
            decision->decide_wayPoint = arrayValue[i]["decide_wayPoint"].asInt();
            decision->out_post_HP_min = arrayValue[i]["out_post_HP_min"].asInt();
            decision->out_post_HP_max = arrayValue[i]["out_post_HP_max"].asInt();
            decision->base_HP_min = arrayValue[i]["base_HP_min"].asInt();
            decision->if_succession = arrayValue[i]["if_succession"].asBool();
            decision->if_reverse = arrayValue[i]["if_reverse"].asBool();
            Json::Value enemyPositionArray = arrayValue[i]["enemyPosition"];
            for (int j = 0; j < int(enemyPositionArray.size()); ++j)
            {
                std::vector<int> temp;
                for (int k = 0; k < int(enemyPositionArray[j].size()); ++k)
                {
                    temp.emplace_back(enemyPositionArray[j][k].asInt());
                }
                decision->enemy_position.emplace_back(temp);
            }
            Json::Value friendPositionArray = arrayValue[i]["friendPosition"];
            for (int j = 0; j < int(friendPositionArray.size()); ++j)
            {
                std::vector<int> temp;
                for (int k = 0; k < int(friendPositionArray[j].size()); ++k)
                {
                    temp.emplace_back(friendPositionArray[j][k].asInt());
                }
                decision->friend_position.emplace_back(temp);
            }
            this->decisions.emplace_back(decision);
        }
        jsonFile.close();
        return true;
    }

    int RobotDecisionSys::checkNowWayPoint(float x, float y)
    {
        RobotPosition pos;
        pos.x = x;
        pos.y = y;
        return this->calculatePosition(pos);
    }

    int RobotDecisionSys::checkNowWayPoint(RobotPosition pos)
    {
        return this->calculatePosition(pos);
    }

    std::shared_ptr<Decision> RobotDecisionSys::decide(int wayPointID, int robot_mode, int _HP, int nowtime, int now_out_post_HP, int now_base_HP, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, std::vector<int> &availableDecisionID, std::map<int, int> &id_pos_f, std::map<int, int> &id_pos_e)
    {
        this->myGameHandler->update(nowtime);
        std::vector<std::shared_ptr<Decision>> tempDecision;
        for (auto &it : friendPositions)
        {
            id_pos_f.insert(std::make_pair(it.robot_id, this->calculatePosition(it)));
        }
        for (auto &it : enemyPositions)
        {
            id_pos_e.insert(std::make_pair(it.robot_id, this->calculatePosition(it)));
        }
        for (auto it : this->decisions)
        {

            if (it->wayPointID[0] != -1)
            {
                bool check = false;
                for (auto &jt : it->wayPointID)
                {
                    check = jt == wayPointID;
                    if (check)
                        break;
                }
                if (!check)
                    continue;
            }
            if ((it->robot_mode != -1 && it->robot_mode != robot_mode) ||
                (it->_maxHP != -1 && _HP > it->_maxHP) ||
                (it->_minHP != -1 && _HP <= it->_minHP) ||
                (it->end_time != -1 && nowtime > it->end_time) ||
                (it->start_time != -1 && nowtime <= it->start_time) ||
                (it->out_post_HP_min != -1 && now_out_post_HP <= it->out_post_HP_min) ||
                (it->out_post_HP_max != -1 && now_out_post_HP > it->out_post_HP_max) ||
                (it->base_HP_min != -1 && now_base_HP < it->base_HP_min))
            {
                continue;
            }
            bool fpFLAG = true;
            for (int i = 0; i < int(it->friend_position.size()); ++i)
            {
                int temp_pos = id_pos_f.find(i)->second;
                if (it->friend_position[i][0] == -1)
                {
                    continue;
                }
                if (std::find(it->friend_position[i].begin(), it->friend_position[i].end(), temp_pos) == it->friend_position[i].end())
                {
                    fpFLAG = false;
                    break;
                }
            }
            bool epFLAG = true;
            for (int i = 0; i < int(it->enemy_position.size()); ++i)
            {
                int temp_pos = id_pos_e.find(i)->second;
                if (it->enemy_position[i][0] == -1)
                {
                    continue;
                }
                if (std::find(it->enemy_position[i].begin(), it->enemy_position[i].end(), temp_pos) == it->enemy_position[i].end())
                {
                    fpFLAG = false;
                    break;
                }
            }
            if (epFLAG && fpFLAG)
                tempDecision.emplace_back(it);
        }
        int max_weight = 0;
        std::shared_ptr<Decision> decision = nullptr;
        for (auto it : tempDecision)
        {
            if (it->weight > max_weight)
            {
                max_weight = it->weight;
                decision = it;
                decision->if_auto = false;
            }
            availableDecisionID.emplace_back(it->id);
        }
        if (decision != nullptr && decision->decide_mode == -1)
            decision->decide_mode = decision->robot_mode;
        if (decision != nullptr && decision->decide_wayPoint == -1)
            decision->decide_wayPoint = wayPointID;
        return decision;
    }

    std::shared_ptr<WayPoint> RobotDecisionSys::getWayPointByID(int id)
    {
        for (auto &it : this->wayPointMap)
        {
            if (it->id == id)
            {
                return it;
            }
        }
        return nullptr;
    }

    std::shared_ptr<Decision> RobotDecisionSys::getDecisionByID(int id)
    {
        for (auto &it : this->decisions)
        {
            if (it->id == id)
            {
                return it;
            }
        }
        return nullptr;
    }

    std::map<int, float> RobotDecisionSys::decideAimTarget(bool _IsBlue, RobotPosition &mypos, std::vector<RobotPosition> &enemyPositions, std::vector<int> &enemyHP, float distance_weight_ratio, float hp_weight_ratio, bool enemyOutpostDown)
    {
        std::map<int, float> result;
        float max_distance = 0.0000001;
        std::map<int, float> distances;
        std::map<int, float> distance_weight;
        for (auto &it : enemyPositions)
        {
            float tempDistance = sqrtf(powf(double(it.x - mypos.x), 2) + powf(double(it.y - mypos.y), 2));
            distances[it.robot_id] = tempDistance;
            if (tempDistance > max_distance)
                max_distance = tempDistance;
        }
        std::map<int, float>::iterator iter_distance_weight;
        for (iter_distance_weight = distances.begin(); iter_distance_weight != distances.end(); iter_distance_weight++)
        {
            distance_weight[iter_distance_weight->first] = (1. - (iter_distance_weight->second / max_distance)) * distance_weight_ratio;
        }
        float max_hp = 0.0000001;
        std::map<int, float> hps;
        std::map<int, float> hp_weight;
        for (int i = 0; i < int(enemyHP.size()); ++i)
        {
            hps[!_IsBlue * CARPOS_NUM + i] = float(enemyHP[i]);
            if (enemyHP[i] > max_hp)
                max_hp = float(enemyHP[i]);
        }
        std::map<int, float>::iterator iter_hp_weight;
        for (iter_hp_weight = hps.begin(); iter_hp_weight != hps.end(); iter_hp_weight++)
        {
            hp_weight[iter_hp_weight->first] = (1. - (iter_hp_weight->second / max_hp)) * hp_weight_ratio;
        }
        for (int i = 0; i < CARPOS_NUM; ++i)
        {
            result[i] = (hp_weight.find(!_IsBlue * CARPOS_NUM + i) != hp_weight.end() ? hp_weight.find(!_IsBlue * CARPOS_NUM + i)->second : -0.5) + (distance_weight.find(!_IsBlue * CARPOS_NUM + i) != distance_weight.end() ? distance_weight.find(!_IsBlue * CARPOS_NUM + i)->second : -0.5) + 0.000001;
        }
        // if (!enemyOutpostDown)
            result[5] = 0;
        return result;
    }

    double RobotDecisionSys::decideAngleByEnemyPos(float _x, float _y, std::vector<RobotPosition> &enemyPositions)
    {
        int index = -1;
        float min_distance = MAXFLOAT;
        for (int i = 0; i < int(enemyPositions.size()); ++i)
        {
            // if(i == 5 || i == 10)
            //     continue;
            if (_x == enemyPositions[i].x && _y == enemyPositions[i].y)
                continue;
            if (enemyPositions[i].x == 0 || enemyPositions[i].y == 0)
                continue;
            float temp_distance = sqrtf(powf(double(enemyPositions[i].x - _x), 2) + powf(double(enemyPositions[i].y - _y), 2));
            if (temp_distance > this->_seek_THR)
                continue;
            double temp_angle = this->calculateAngle(_x, _y, enemyPositions[i].x, enemyPositions[i].y);
            // if (this->checkBlock(this->transformPoint(_x, _y, this->_REAL_WIDTH, this->_REAL_HEIGHT, int((this->_REAL_WIDTH / this->_REAL_HEIGHT) * 1080), 1080), temp_angle, int(temp_distance * (1080. / this->_REAL_HEIGHT))))
            //     continue;
            if (temp_distance < min_distance)
            {
                min_distance = temp_distance;
                index = i;
            }
            
        }
        return index == -1 ? -1 : this->calculateAngle(_x, _y, enemyPositions[index].x, enemyPositions[index].y);
    }

    double RobotDecisionSys::calculateAngle(double x1, double y1, double x2, double y2)
    {
        if (x1 == x2 && y1 == y2)
            return 0;
        double angle_temp;
        double xx, yy;
        xx = x2 - x1;
        yy = y2 - y1;
        if (xx == 0.0)
            angle_temp = CV_PI / 2.0;
        else
            angle_temp = atan(fabs(yy / xx));
        if ((xx < 0.0) && (yy >= 0.0))
            angle_temp = CV_PI - angle_temp;
        else if ((xx < 0.0) && (yy < 0.0))
            angle_temp = CV_PI + angle_temp;
        else if ((xx >= 0.0) && (yy < 0.0))
            angle_temp = CV_PI * 2.0 - angle_temp;
        return angle_temp;
    }

    double RobotDecisionSys::calculateAngle(cv::Point2d p1, cv::Point2d p2)
    {
        if (p1.x == p2.x && p1.y == p2.y)
            return 0;
        double angle_temp;
        double xx, yy;
        xx = p2.x - p1.x;
        yy = p2.y - p1.y;
        if (xx == 0.0)
            angle_temp = CV_PI / 2.0;
        else
            angle_temp = atan(fabs(yy / xx));
        if ((xx < 0.0) && (yy >= 0.0))
            angle_temp = CV_PI - angle_temp;
        else if ((xx < 0.0) && (yy < 0.0))
            angle_temp = CV_PI + angle_temp;
        else if ((xx >= 0.0) && (yy < 0.0))
            angle_temp = CV_PI * 2.0 - angle_temp;
        return angle_temp;
    }

    float RobotDecisionSys::getDistanceTHR()
    {
        return this->_distance_THR;
    }

    void RobotDecisionSys::setDistanceTHR(float thr)
    {
        this->_distance_THR = thr;
    }

    float RobotDecisionSys::getSeekTHR()
    {
        return this->_seek_THR;
    }

    void RobotDecisionSys::setSeekTHR(float thr)
    {
        this->_seek_THR = thr;
    }

    void RobotDecisionSys::UpdateDecisionMap(int &activateDecisionID, std::vector<int> &availableDecisionID, int &nowWayPoint, double yaw, cv::Point2f car_center, double car_orientation, double aim_yaw, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, std::map<int, int> &id_pos_f, std::map<int, int> &id_pos_e)
    {
        cv::Mat dstMap;
        if (IfShowUI)
        {
            if (!IfUIInited)
            {
                cv::namedWindow("DecisionMapUI", cv::WindowFlags::WINDOW_NORMAL);
                this->IfUIInited = true;
            }
        }
        else
        {
            return;
        }
        car_orientation = car_orientation == -1 ? yaw : car_orientation;
        cv::Point dst_car_center = this->transformPoint(car_center, this->_REAL_WIDTH, this->_REAL_HEIGHT, int((this->_REAL_WIDTH / this->_REAL_HEIGHT) * 1080), 1080);
        dstMap = this->decisionMap.clone();
        int activateWayPointID = this->getDecisionByID(activateDecisionID)->decide_wayPoint;
        std::vector<int> availableWayPointID;
        for (auto &it : availableDecisionID)
        {
            availableWayPointID.emplace_back(this->getDecisionByID(it)->decide_wayPoint);
        }
        for (int i = 0; i < int(this->wayPointMap.size()); ++i)
        {
            int temp_id = this->wayPointMap[i]->id;
            cv::Point dst_way_point_center = this->transformPoint(this->wayPointMap[i]->x, this->wayPointMap[i]->y, this->_REAL_WIDTH, this->_REAL_HEIGHT, int((this->_REAL_WIDTH / this->_REAL_HEIGHT) * 1080), 1080);
            if (std::find_if(id_pos_f.begin(), id_pos_f.end(), [temp_id](const auto &item)
                             { return item.second == temp_id; }) != id_pos_f.end())
            {
                cv::rectangle(dstMap, cv::Point(dst_way_point_center.x - 28, dst_way_point_center.y - 15), cv::Point(dst_way_point_center.x + 28, dst_way_point_center.y + 15), cv::Scalar(0, 255, 0), -1);
            }
            if (std::find_if(id_pos_e.begin(), id_pos_e.end(), [temp_id](const auto &item)
                             { return item.second == temp_id; }) != id_pos_e.end())
            {
                cv::rectangle(dstMap, cv::Point(dst_way_point_center.x - 15, dst_way_point_center.y - 28), cv::Point(dst_way_point_center.x + 15, dst_way_point_center.y + 28), cv::Scalar(0, 0, 255), -1);
            }
            if (temp_id == activateWayPointID)
            {
                this->drawWayPoint(dstMap, dst_way_point_center, temp_id, 2);
            }
            else if (temp_id == nowWayPoint)
            {
                this->drawWayPoint(dstMap, dst_way_point_center, temp_id, 3);
            }
            else
            {
                bool check_flag = false;
                for (auto &it : availableWayPointID)
                {
                    if (it == temp_id)
                    {
                        check_flag = true;
                        break;
                    }
                }
                this->drawWayPoint(dstMap, dst_way_point_center, temp_id, check_flag ? 1 : 0);
            }
        }
        this->drawCar(dstMap, dst_car_center, car_orientation, yaw, aim_yaw);
        cv::circle(dstMap, dst_car_center, int(this->_seek_THR / float(this->_REAL_HEIGHT / 1080)), cv::Scalar(255, 0, 0), 1);
        for (auto &it : friendPositions)
        {
            this->drawOthCar(dstMap, this->transformPoint(it.x, it.y, this->_REAL_WIDTH, this->_REAL_HEIGHT, int((this->_REAL_WIDTH / this->_REAL_HEIGHT) * 1080), 1080), it.robot_id, 0);
        }
        for (auto &it : enemyPositions)
        {
            this->drawOthCar(dstMap, this->transformPoint(it.x, it.y, this->_REAL_WIDTH, this->_REAL_HEIGHT, int((this->_REAL_WIDTH / this->_REAL_HEIGHT) * 1080), 1080), it.robot_id, 1);
        }
        cv::imshow("DecisionMapUI", dstMap);
        cv::resizeWindow("DecisionMapUI", cv::Size(1280, 720));
        cv::waitKey(1);
    }

    void RobotDecisionSys::drawWayPoint(cv::Mat &img, cv::Point2i center, int id, int type)
    {
        cv::Scalar color;
        cv::Scalar color_txt;
        switch (type)
        {
        case 0:
            color = cv::Scalar(135, 138, 128);
            color_txt = cv::Scalar(255, 248, 248);
            break;
        case 1:
            color = cv::Scalar(230, 224, 176);
            color_txt = cv::Scalar(255, 105, 65);
            break;
        case 2:
            color = cv::Scalar(0, 225, 0);
            color_txt = cv::Scalar(20, 128, 48);
            break;
        case 3:
            color = cv::Scalar(0, 255, 255);
            color_txt = cv::Scalar(18, 153, 255);
            break;

        default:
            break;
        }
        cv::circle(img, center, 24, color_txt, -1);
        cv::circle(img, center, 22, color, -1);
        cv::Size text_size = cv::getTextSize(std::to_string(id), cv::FONT_HERSHEY_SIMPLEX, 1, 2, 0);
        cv::putText(img, std::to_string(id), cv::Point2i(center.x - int(text_size.width / 2), center.y + int(text_size.height / 2)), cv::FONT_HERSHEY_SIMPLEX, 1, color_txt, 2);
    }

    cv::Point2i RobotDecisionSys::transformPoint(float _x, float _y, float width, float height, int img_cols, int img_rows)
    {
        return cv::Point2i(int(double(_x / width) * img_cols), int(double(_y / height) * img_rows));
    }

    cv::Point2i RobotDecisionSys::transformPoint(cv::Point2f center, float width, float height, int img_cols, int img_rows)
    {
        return cv::Point2i(int(double(center.x / width) * img_cols), int(double(center.y / height) * img_rows));
    }

    void RobotDecisionSys::drawCar(cv::Mat &img, cv::Point2i center, double &car_orientation, double &yaw, double &aim_yaw)
    {
        if (yaw == -1)
        {
            yaw = car_orientation;
        }
        cv::Size wh = cv::Size(50, 35);
        cv::Point point_L_U = cv::Point(center.x - wh.width / 2, center.y - wh.height / 2);
        cv::Point point_R_U = cv::Point(center.x + wh.width / 2, center.y - wh.height / 2);
        cv::Point point_R_L = cv::Point(center.x + wh.width / 2, center.y + wh.height / 2);
        cv::Point point_L_L = cv::Point(center.x - wh.width / 2, center.y + wh.height / 2);
        cv::Point point[4] = {point_L_U, point_R_U, point_R_L, point_L_L};
        cv::Point after_point[4] = {cv::Point(0, 0)};
        for (int i = 0; i < 4; ++i)
        {
            int x = point[i].x - center.x;
            int y = point[i].y - center.y;
            after_point[i].x = cvRound(x * cos(-car_orientation) + y * sin(-car_orientation) + center.x);
            after_point[i].y = cvRound(-x * sin(-car_orientation) + y * cos(-car_orientation) + center.y);
        }
        for (int i = 0; i < 3; ++i)
        {
            cv::line(img, after_point[i], after_point[i + 1], cv::Scalar(0, 255, 0), 3);
            if (i == 2)
            {
                cv::line(img, after_point[i + 1], after_point[0], cv::Scalar(0, 255, 0), 3);
            }
        }
        int seek_radius = int(this->_seek_THR / float(this->_REAL_HEIGHT / 1080));
        cv::Point2i Car_End = this->createEndPointByTheta(center, car_orientation, 35);
        cv::Point2i PAN_End = this->createEndPointByTheta(center, yaw, int(seek_radius / 3));
        if (aim_yaw < 0. && aim_yaw != -1.)
        {
            aim_yaw = 2. * CV_PI - abs(aim_yaw);
        }
        cv::Point2i AIM_YAW_End = this->createEndPointByTheta(center, aim_yaw, seek_radius);
        cv::Point2i Seek_End_s = this->createEndPointByTheta(center, yaw + (this->_CAR_SEEK_FOV / 2.) * CV_PI / 180., seek_radius);
        cv::Point2i Seek_End_e = this->createEndPointByTheta(center, yaw - (this->_CAR_SEEK_FOV / 2.) * CV_PI / 180., seek_radius);
        cv::line(img, center, Car_End, cv::Scalar(255, 255, 0), 3);
        cv::line(img, center, Seek_End_s, cv::Scalar(0, 255, 255), 2);
        cv::line(img, center, Seek_End_e, cv::Scalar(0, 255, 255), 2);
        if (aim_yaw != -1.)
            cv::line(img, center, AIM_YAW_End, cv::Scalar(255, 100, 0), 2);
        cv::line(img, center, PAN_End, cv::Scalar(0, 0, 255), 2);
    }

    void RobotDecisionSys::drawOthCar(cv::Mat &img, cv::Point2i center, int &id, int type)
    {
        float w = 35;
        cv::Scalar color;
        switch (type)
        {
        case 0:
            color = cv::Scalar(0, 255, 0);
            break;
        case 1:
            color = cv::Scalar(0, 0, 255);
            break;

        default:
            break;
        }
        cv::Point _t = cv::Point(center.x, center.y - int((w / 2.) / cos(30. * CV_PI / 180.)));
        cv::Point _l = cv::Point(center.x - w / 2, center.y + int((w / 2.) * tan(30. * CV_PI / 180.)));
        cv::Point _r = cv::Point(center.x + w / 2, center.y + int((w / 2.) * tan(30. * CV_PI / 180.)));
        cv::line(img, _t, _l, color, 2);
        cv::line(img, _r, _l, color, 2);
        cv::line(img, _t, _r, color, 2);
        cv::Size text_size = cv::getTextSize(std::to_string(id), cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, 0);
        cv::putText(img, std::to_string(id), cv::Point2i(center.x - int(text_size.width / 2), center.y + int(text_size.height / 2)), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }

    bool RobotDecisionSys::getIfShowUI()
    {
        return this->IfShowUI;
    }

    void RobotDecisionSys::setIfShowUI(bool ifShowUI)
    {
        this->IfShowUI = ifShowUI;
    }
}
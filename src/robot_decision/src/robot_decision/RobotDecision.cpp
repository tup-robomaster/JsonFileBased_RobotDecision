#include "../../include/robot_decision/RobotDecision.h"
#include "../../include/Json/json.h"

namespace rdsys
{
    int RobotDecisionSys::calculatePosition(RobotPosition &pos)
    {
        float distance = FLT_MAX;
        int id = -1;
        for (auto it : this->wayPointMap)
        {
            float tempDistance = std::sqrt((it->x - pos.x) * (it->x - pos.x) + (it->y - pos.y) * (it->y - pos.y));
            if (tempDistance < distance)
            {
                distance = tempDistance;
                id = it->id;
            }
        }
        if (distance - this->_distance_THR > 0)
        {
            id = -1;
        }
        return id;
    }

    std::vector<int> RobotDecisionSys::calculatePath(int startWapPointID, int endWapPointID)
    {
        std::vector<std::vector<int>> paths;
        std::vector<int> result;
        bool flag = true;
        for (int i = 0; i < int(connectionList.size()) && flag; ++i)
        {
            if (paths.size() == 0)
            {
                for (int &it : this->connectionList[startWapPointID])
                {
                    paths.emplace_back(std::vector<int>(1, it));
                }
            }
            else
            {
                for (int j = 0; j < int(paths.size()) && flag; ++j)
                {
                    int count = 0;
                    for (int &it : this->connectionList[paths[i][paths.size() - 1]])
                    {
                        if (this->connectionList[paths[i][paths.size() - 1]].size() > 1 && it == this->connectionList[paths[i][paths.size() - 1]][this->connectionList[paths[i][paths.size() - 1]].size() - 1])
                        {
                            continue;
                        }
                        if (count == 0)
                        {
                            paths[i].emplace_back(it);
                            if (it == endWapPointID)
                            {
                                result.swap(paths[i]);
                                flag = false;
                            }
                        }
                        else
                        {
                            std::vector<int> temp(paths[i]);
                            temp.emplace_back(it);
                            paths.push_back(temp);
                            if (it == endWapPointID)
                            {
                                result.swap(temp);
                                flag = false;
                            }
                        }
                        ++count;
                    }
                }
            }
        }
        return result;
    }

    cv::Point2i RobotDecisionSys::createEndPointByTheta(double x1, double y1, double theta, int length)
    {
        return cv::Point2i((int)round(x1 + length * cos(theta)), (int)round(y1 + length * sin(theta)));
    }

    cv::Point2i RobotDecisionSys::createEndPointByTheta(cv::Point start, double theta, int length)
    {
        return cv::Point2i((int)round(start.x + length * cos(theta)), (int)round(start.y + length * sin(theta)));
    }

    RobotDecisionSys::RobotDecisionSys(float &_distance_THR, float &_seek_THR)
    {
        this->_distance_THR = _distance_THR;
        this->_seek_THR = _seek_THR;
    }

    RobotDecisionSys::~RobotDecisionSys()
    {
    }

    bool RobotDecisionSys::decodeWayPoints(char *filePath)
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
            WayPoint *wayPoint = new WayPoint();
            std::vector<int> connect;
            wayPoint->id = arrayValue[i]["id"].asInt();
            wayPoint->type = arrayValue[i]["type"].asInt();
            wayPoint->x = arrayValue[i]["x"].asFloat();
            wayPoint->y = arrayValue[i]["y"].asFloat();
            wayPoint->theta = arrayValue[i]["angle"].asDouble();
            Json::Value connectedPoints = arrayValue[i]["connect"];
            for (int j = 0; j < int(connectedPoints.size()); ++j)
            {
                connect.emplace_back(connectedPoints[j].asInt());
            }
            Json::Value enemyWeightsArray = arrayValue[i]["enemyWeights"];
            for (int j = 0; j < int(enemyWeightsArray.size()); ++j)
            {
                wayPoint->enemyWeights[j] = enemyWeightsArray[j].asInt();
            }
            this->wayPointMap.emplace_back(wayPoint);
            this->connectionList.emplace_back(connect);
        }
        jsonFile.close();
        return true;
    }

    bool RobotDecisionSys::decodeDecisions(char *filePath)
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
            Decision *decision = new Decision();
            decision->id = arrayValue[i]["id"].asInt();
            decision->name = arrayValue[i]["name"].asCString();
            decision->wayPointID = arrayValue[i]["wayPointID"].asInt();
            decision->weight = arrayValue[i]["weight"].asInt();
            decision->start_time = arrayValue[i]["start_time"].asInt();
            decision->end_time = arrayValue[i]["end_time"].asInt();
            decision->robot_mode = arrayValue[i]["robot_mode"].asInt();
            decision->_minHP = arrayValue[i]["minHP"].asInt();
            decision->_maxHP = arrayValue[i]["maxHP"].asInt();
            decision->decide_mode = arrayValue[i]["decide_mode"].asInt();
            decision->decide_wayPoint = arrayValue[i]["decide_wayPoint"].asInt();
            decision->out_post_HP_max = arrayValue[i]["out_post_HP_max"].asInt();
            decision->if_succession = arrayValue[i]["if_succession"].asBool();
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

    std::shared_ptr<Decision> RobotDecisionSys::decide(int wayPointID, int robot_mode, int _HP, int nowtime, int now_out_post_HP, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, std::vector<int> &availableDecisionID)
    {
        std::vector<std::shared_ptr<Decision>> tempDecision;
        std::map<int, int> id_pos_f;
        for (auto &it : friendPositions)
        {
            id_pos_f[it.robot_id] = this->calculatePosition(it);
        }
        std::map<int, int> id_pos_e;
        for (auto &it : enemyPositions)
        {
            id_pos_e[it.robot_id] = this->calculatePosition(it);
        }
        for (auto it : this->decisions)
        {
            if ((it->wayPointID != wayPointID || it->robot_mode != robot_mode) && it->wayPointID != -1 && it->robot_mode != -1)
            {
                continue;
            }
            else if (it->_maxHP != -1 && _HP > it->_maxHP)
            {
                continue;
            }
            else if (it->_minHP != -1 && _HP <= it->_maxHP)
            {
                continue;
            }
            else if (it->end_time != -1 && nowtime > it->end_time)
            {
                continue;
            }
            else if (it->start_time != -1 && nowtime <= it->start_time)
            {
                continue;
            }
            else if (it->out_post_HP_max != -1 && now_out_post_HP < it->out_post_HP_max)
            {
                continue;
            }
            bool fpFLAG = false;
            for (int i = 0; i < int(it->friend_position.size()); ++i)
            {
                int size = it->friend_position[i].size();
                for (int j = 0; j < size; ++j)
                {
                    if (it->friend_position[i][j] == -1)
                    {
                        fpFLAG = true;
                        break;
                    }
                }
                if (fpFLAG)
                    break;
                if (size == 0)
                {
                    continue;
                }
                else if (0 == id_pos_f.count(i))
                {
                    continue;
                }
                int temp_pos;
                temp_pos = id_pos_f[i];
                for (int j = 0; j < size; ++j)
                {
                    if (it->friend_position[i][j] == temp_pos)
                    {
                        fpFLAG = true;
                        break;
                    }
                }
                if (fpFLAG)
                    break;
            }
            bool epFLAG = false;
            for (int i = 0; i < int(it->enemy_position.size()); ++i)
            {
                int size = it->enemy_position[i].size();
                for (int j = 0; j < size; ++j)
                {
                    if (it->enemy_position[i][0] == -1)
                    {
                        epFLAG = true;
                        break;
                    }
                }
                if (epFLAG)
                    break;
                if (size == 0)
                {
                    continue;
                }
                else if (0 == id_pos_e.count(i))
                {
                    continue;
                }
                int temp_pos;
                temp_pos = id_pos_e[i];
                for (int j = 0; j < size; ++j)
                {
                    if (it->enemy_position[i][j] == temp_pos)
                        epFLAG = true;
                    break;
                }
                if (epFLAG)
                    break;
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
            decision->decide_wayPoint = decision->wayPointID;
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

    int RobotDecisionSys::decideAimTarget(RobotPosition &mypos, std::vector<RobotPosition> &enemyPositions, std::vector<int> &detectedEnemy, int &myWayPointID)
    {
        std::map<int, float> distances;
        for (auto &it : enemyPositions)
        {
            float tempDistance = std::sqrt((it.x - mypos.x) * (it.x - mypos.x) + (it.y - mypos.y) * (it.y - mypos.y));
            distances[it.robot_id] = tempDistance;
        }
        std::shared_ptr<WayPoint> myWayPoint = this->getWayPointByID(myWayPointID);
        if (myWayPoint == nullptr)
            return -1;
        int baseWeight = 0;
        int selectId = -1;
        for (auto &it : detectedEnemy)
        {
            auto iter = myWayPoint->enemyWeights.find(it);
            if (iter != myWayPoint->enemyWeights.end())
            {
                if (iter->second > baseWeight)
                {
                    selectId = iter->first;
                    baseWeight = iter->second;
                }
                else if (iter->second == baseWeight)
                {
                    auto iterA = distances.find(selectId);
                    auto iterB = distances.find(iter->first);
                    if (iterA != distances.end() && iterB != distances.end() && iterB->second - iterA->second < 0)
                    {
                        selectId = iter->first;
                        baseWeight = iter->second;
                    }
                }
            }
        }
        return selectId;
    }

    double RobotDecisionSys::decideAngleByEnemyPos(float _x, float _y, std::vector<RobotPosition> &enemyPositions)
    {
        int index = -1;
        float min_distance = MAXFLOAT;
        for (int i = 0; i < int(enemyPositions.size()); ++i)
        {
            if (_x == enemyPositions[i].x && _y == enemyPositions[i].y)
                continue;
            if (enemyPositions[i].x == 0 || enemyPositions[i].y == 0)
                continue;
            float temp_distance = std::sqrt((enemyPositions[i].x - _x) * (enemyPositions[i].x - _x) + (enemyPositions[i].y - _y) * (enemyPositions[i].y - _y));
            if (temp_distance < min_distance)
            {
                min_distance = temp_distance;
                index = i;
            }
        }
        return (index == -1 || min_distance > this->_seek_THR) ? -1 : this->calculateAngle(_x, _y, enemyPositions[index].x, enemyPositions[index].y);
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
        return (angle_temp);
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

    void RobotDecisionSys::UpdateDecisionMap(int &activateDecisionID, std::vector<int> &availableDecisionID, int &nowWayPoint, double yaw, cv::Point car_center, double car_orientation)
    {
        if (IfShowUI)
        {
            if (!IfUIInited)
            {
                this->decisionMap = cv::Mat::zeros(1080, int((REAL_WIDTH / REAL_HEIGHT) * 1080), CV_8UC3);
                cv::namedWindow("DecisionMapUI", cv::WindowFlags::WINDOW_NORMAL);
                this->IfUIInited = true;
            }
        }
        else
        {
            return;
        }
        this->decisionMap = cv::Mat::zeros(1080, int((REAL_WIDTH / REAL_HEIGHT) * 1080), CV_8UC3);
        int activateWayPointID = this->getDecisionByID(activateDecisionID)->decide_wayPoint;
        std::vector<int> availableWayPointID;
        for (auto &it : availableDecisionID)
        {
            availableWayPointID.emplace_back(this->getDecisionByID(it)->decide_wayPoint);
        }
        for (int i = 0; i < int(this->wayPointMap.size()); ++i)
        {
            int temp_id = this->wayPointMap[i]->id;
            if (temp_id == activateWayPointID)
            {
                this->drawWayPoint(this->decisionMap, this->transformPoint(this->wayPointMap[i]->x, this->wayPointMap[i]->y, REAL_WIDTH, REAL_HEIGHT, int((REAL_WIDTH / REAL_HEIGHT) * 1080), 1080), temp_id, 2);
            }
            else if (temp_id == nowWayPoint)
            {
                this->drawWayPoint(this->decisionMap, this->transformPoint(this->wayPointMap[i]->x, this->wayPointMap[i]->y, REAL_WIDTH, REAL_HEIGHT, int((REAL_WIDTH / REAL_HEIGHT) * 1080), 1080), temp_id, 3);
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
                this->drawWayPoint(this->decisionMap, this->transformPoint(this->wayPointMap[i]->x, this->wayPointMap[i]->y, REAL_WIDTH, REAL_HEIGHT, int((REAL_WIDTH / REAL_HEIGHT) * 1080), 1080), temp_id, check_flag ? 1 : 0);
            }
        }
        this->drawCar(this->decisionMap, this->transformPoint(car_center, REAL_WIDTH, REAL_HEIGHT, int((REAL_WIDTH / REAL_HEIGHT) * 1080), 1080), car_orientation, yaw);
        cv::imshow("DecisionMapUI", this->decisionMap);
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
        return cv::Point2i(int((_x / width) * img_cols), int((_y / height) * img_rows));
    }

    cv::Point2i RobotDecisionSys::transformPoint(cv::Point center, float width, float height, int img_cols, int img_rows)
    {
        return cv::Point2i(int((center.x / width) * img_cols), int((center.y / height) * img_rows));
    }

    void RobotDecisionSys::drawCar(cv::Mat &img, cv::Point2i center, double car_orientation, double yaw)
    {
        if(yaw == -1)
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
            after_point[i].x = cvRound(x * cos(car_orientation) + y * sin(car_orientation) + center.x);
            after_point[i].y = cvRound(-x * sin(car_orientation) + y * cos(car_orientation) + center.y);
        }
        for (int j = 0; j < 3; ++j)
        {
            cv::line(img, after_point[j], after_point[j + 1], cv::Scalar(0, 255, 0), 3);
            if (j == 2)
            {
                cv::line(img, after_point[j + 1], after_point[0], cv::Scalar(0, 255, 0), 3);
            }
        }
        cv::Point2i Car_End = this->createEndPointByTheta(center, car_orientation, 35);
        cv::Point2i PAN_End = this->createEndPointByTheta(center, yaw, 50);
        cv::line(img, center, Car_End, cv::Scalar(255, 255, 0), 3);
        cv::line(img, center, PAN_End, cv::Scalar(0, 0, 255), 3);
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
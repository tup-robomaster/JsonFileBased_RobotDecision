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

    std::shared_ptr<Decision> RobotDecisionSys::decide(int wayPointID, int robot_mode, int _HP, int nowtime, int now_out_post_HP, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions)
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
                continue;
            if (it->_maxHP != -1 && _HP > it->_maxHP)
                continue;
            if (it->_minHP != -1 && _HP <= it->_maxHP)
                continue;
            if (it->end_time != -1 && nowtime > it->end_time)
                continue;
            if (it->start_time != -1 && nowtime <= it->start_time)
                continue;
            if (it->out_post_HP_max != -1 && now_out_post_HP < it->out_post_HP_max)
                continue;
            bool fpFLAG = false;
            for (int i = 0; i < int(it->friend_position.size()); ++i)
            {
                int size = it->friend_position[i].size();
                if (size == 0)
                    continue;
                if (0 == id_pos_f.count(i + 1))
                    continue;
                int temp_pos;
                temp_pos = id_pos_f[i + 1];
                for (int j = 0; j < size; ++j)
                {
                    if (it->friend_position[i][j] == temp_pos || it->friend_position[i][j] == -1)
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
                if (size == 0)
                    continue;
                if (0 == id_pos_e.count(i + 1))
                    continue;
                int temp_pos;
                temp_pos = id_pos_e[i + 1];
                for (int j = 0; j < size; ++j)
                {
                    if (it->enemy_position[i][j] == temp_pos || it->enemy_position[i][j] == -1)
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
        std::shared_ptr<Decision> decision;
        for (auto it : tempDecision)
        {
            if (it->weight > max_weight)
            {
                max_weight = it->weight;
                decision = it;
                decision->if_auto = false;
            }
        }
        if (decision->decide_mode == -1)
            decision->decide_mode = decision->robot_mode;
        if (decision->decide_wayPoint == -1)
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
        if(myWayPoint == nullptr)
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
        double angle_temp;
        double xx, yy;
        xx = x2 - x1;
        yy = y2 - y1;
        if (xx == 0.0)
            angle_temp = _PI / 2.0;
        else
            angle_temp = atan(fabs(yy / xx));
        if ((xx < 0.0) && (yy >= 0.0))
            angle_temp = _PI - angle_temp;
        else if ((xx < 0.0) && (yy < 0.0))
            angle_temp = _PI + angle_temp;
        else if ((xx >= 0.0) && (yy < 0.0))
            angle_temp = _PI * 2.0 - angle_temp;
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
}

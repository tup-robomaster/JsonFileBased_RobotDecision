#include "../../include/robot_decision/RobotDecision.h"
#include "../../include/Json/json.h"

namespace robotdecisionsystem
{
    int RobotDecisionSys::calculatePosition(RobotPosition &pos)
    {
        float distance = FLT_MAX;
        int id = -1;
        for (auto it : this->wayPointMap)
        {
            float tempDistance = std::sqrt((it.x - pos.x) * (it.x - pos.x) + (it.y - pos.y) * (it.y - pos.y));
            if (tempDistance < distance)
            {
                distance = tempDistance;
                id = it.id;
            }
        }
        return id;
    }

    RobotDecisionSys::RobotDecisionSys()
    {
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
        std::vector<std::vector<int>> tempconnectionList;
        for (int i(0); i < int(arrayValue.size()); ++i)
        {
            WayPoint wayPoint;
            std::vector<int> connect;
            wayPoint.id = arrayValue[i]["id"].asInt();
            wayPoint.type = arrayValue[i]["type"].asInt();
            wayPoint.x = arrayValue[i]["x"].asFloat();
            wayPoint.y = arrayValue[i]["y"].asFloat();
            Json::Value connectedPoints = arrayValue[i]["connect"];
            for (int j(0); j < int(connectedPoints.size()); ++j)
            {
                connect.emplace_back(connectedPoints[j].asInt());
            }
            this->wayPointMap.push_back(wayPoint);
            this->connectionList.push_back(connect);
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
        for (int i(0); i < int(arrayValue.size()); ++i)
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
            decision->if_succession = arrayValue[i]["if_succession"].asBool();
            Json::Value enemyPositionArray = arrayValue[i]["enemyPosition"];
            for (int j(1); j < int(enemyPositionArray.size()); ++j)
            {
                std::vector<int> temp;
                for (int k(0); k < int(enemyPositionArray[char(j)].size()); ++k)
                {
                    temp.emplace_back(enemyPositionArray[char(j)][k].asInt());
                }
                decision->enemy_position.emplace_back(temp);
            }
            Json::Value friendPositionArray = arrayValue[i]["friendPosition"];
            for (int j(1); j < int(friendPositionArray.size()); ++j)
            {
                std::vector<int> temp;
                for (int k(0); k < int(friendPositionArray[char(j)].size()); ++k)
                {
                    temp.emplace_back(friendPositionArray[char(j)][k].asInt());
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

    Decision *RobotDecisionSys::decide(int wayPointID, int robot_mode, int _HP, int nowtime, std::vector<RobotPosition> friendPositions, std::vector<RobotPosition> enemyPositions)
    {
        std::vector<Decision *> tempDecision;
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
            if (it->wayPointID != wayPointID || it->robot_mode != robot_mode)
                continue;
            if (it->_maxHP != -1 && _HP > it->_maxHP)
                continue;
            if (it->_minHP != -1 && _HP <= it->_maxHP)
                continue;
            if (it->end_time != -1 && nowtime > it->end_time)
                continue;
            if (it->start_time != -1 && nowtime <= it->start_time)
                continue;
            bool fpFLAG = false;
            for (int i(0); i < int(it->friend_position.size()); ++i)
            {
                int size = it->friend_position[i].size();
                if (size == 0)
                    continue;
                if (0 == id_pos_f.count(char(i + 1)))
                    continue;
                int temp_pos;
                temp_pos = id_pos_f[char(i + 1)];
                for (int j(0); j < size; ++j)
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
            for (int i(0); i < int(it->enemy_position.size()); ++i)
            {
                int size = it->enemy_position[i].size();
                if (size == 0)
                    continue;
                if (0 == id_pos_e.count(char(i + 1)))
                    continue;
                int temp_pos;
                temp_pos = id_pos_e[char(i + 1)];
                for (int j(0); j < size; ++j)
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
        Decision *decision;
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

    WayPoint RobotDecisionSys::getWayPointByID(int id)
    {
        for (auto &it : this->wayPointMap)
        {
            if (it.id == id)
            {
                return it;
            }
        }
        return WayPoint();
    }

    Decision *RobotDecisionSys::getDecisionByID(int id)
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
}

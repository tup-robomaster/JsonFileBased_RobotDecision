#ifndef _RD_STRUCTS_H
#define _RD_STRUCTS_H

#include "./public.h"

namespace rdsys
{
    typedef struct RobotPosition
    {
        int robot_id = -1;
        float x;
        float y;
        RobotPosition(){};
        RobotPosition(int id, float x, float y)
        {
            this->robot_id = id;
            this->x = x;
            this->y = y;
        };
    } RobotPosition;

    typedef struct WayPoint
    {
        int id;
        int type;
        float x;
        float y;
        double theta;
        std::map<int, int> enemyWeights;
    } WayPoint;

    typedef struct Decision
    {
        bool if_auto = true;
        int id;
        const char *name;
        int wayPointID;
        int weight;
        // condition
        int robot_mode;
        int start_time;
        int end_time;
        int _minHP;
        int _maxHP;
        std::vector<std::vector<int>> enemy_position;
        std::vector<std::vector<int>> friend_position;
        // decision
        int decide_mode;
        int decide_wayPoint;
        bool if_succession;
    } Decision;
}

#endif
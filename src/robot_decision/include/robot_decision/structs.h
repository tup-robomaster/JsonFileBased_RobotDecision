#ifndef _RD_STRUCTS_H
#define _RD_STRUCTS_H

#include "./public.h"

namespace rdsys
{
    /**
     * @brief 机器人位置信息
     */
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

    /**
     * @brief 路径点信息
     */
    typedef struct WayPoint
    {
        int id;
        int type;
        float x;
        float y;
        double theta;
        std::map<int, int> enemyWeights;
        std::vector<int> connection;
    } WayPoint;

    /**
     * @brief 决策信息
     */
    typedef struct Decision
    {
        bool if_auto = true;
        int id;
        const char *name;
        std::vector<int> wayPointID;
        int weight;
        // condition
        int robot_mode;
        int start_time;
        int end_time;
        int _minHP;
        int _maxHP;
        int out_post_HP_min;
        int out_post_HP_max;
        int base_HP_min;
        std::vector<std::vector<int>> enemy_position;
        std::vector<std::vector<int>> friend_position;
        // decision
        int decide_mode;
        int decide_wayPoint;
        bool if_succession;
        bool if_reverse;
    } Decision;

    enum Mode
    {
        AUTOAIM = 8,
        MANUAL_ATTACK = 8,
        MANUAL_BACKDEFENSE = 11
    };

    enum GameStage
    {
        COMPETITON_NOT_STARTED = 0,
        PREPARATION_STAGE = 1,
        SELF_INSPECTION_STAGE = 2,
        FIVE_S_COUNTDOWN = 3,
        IN_BATTLE = 4,
        END = 5
    };
}

#endif
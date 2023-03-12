#ifndef _RD_DECISION_H
#define _RD_DECISION_H

#include "./structs.h"

namespace rdsys
{
#define _PI 3.1415926
    enum class DecisonState
    {
        SUCCESS,
        EXCETING,
        FAILURE
    };
    /**
     * @brief 决策系统类
     * 提供机器人决策相关处理接口
     */
    class RobotDecisionSys
    {
    private:
        std::vector<WayPoint *> wayPointMap;
        std::vector<std::vector<int>> connectionList;
        std::vector<Decision *> decisions;

        float _distance_THR = 0;
        float _seek_THR = 5.0;

    private:
        /**
         * @brief 计算机器人当前所在路径点
         * @param pos
         * 当前机器人坐标信息
         */
        int calculatePosition(RobotPosition &pos);
        /**
         * @brief 计算路径路线
         * @param startWapPointID
         * 起点路径点ID
         * @param endWapPointID
         * 终点路径点ID
         */
        std::vector<int> calculatePath(int startWapPointID, int endWapPointID);

        double calculateAngle(double x1, double y1, double x2, double y2);

    public:
        RobotDecisionSys(float &_distance_THR, float &_seek_THR);
        ~RobotDecisionSys();

        /**
         * @brief 解码路径点Json文件
         * @param filePath
         * 路径点Json文件地址
         */
        bool decodeWayPoints(char *filePath);
        /**
         * @brief 解码决策Json文件
         * @param filePath
         * 决策Json文件地址
         */
        bool decodeDecisions(char *filePath);

        /**
         * @brief 检查机器人当前所在路径点
         * @param x
         * 机器人坐标x
         * @param y
         * 机器人坐标y
         */
        int checkNowWayPoint(float x, float y);
        /**
         * @brief 检查机器人当前所在路径点
         * @param pos
         * 当前机器人坐标信息
         */
        int checkNowWayPoint(RobotPosition pos);
        /**
         * @brief 机器人决策
         * @param wayPointID
         * 当前机器人路径点ID
         * @param robot_mode
         * 当前机器人模式
         * @param _HP
         * 当前机器人血量
         * @param now
         * 当前比赛剩余时间
         * @param friendPositions
         * 友方机器人位置
         * @param enemtPositions
         * 敌方机器人位置
         */
        Decision *decide(int wayPointID, int robot_mode, int _HP, int nowtime, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions);

        /**
         * @brief 根据ID获取路径点
         * @param id
         * 路径点ID
         */
        WayPoint *getWayPointByID(int id);
        /**
         * @brief 根据ID获取决策
         * @param id
         * 决策ID
         */
        Decision *getDecisionByID(int id);

        /**
         * @brief 决策打击目标
         * @param mypos
         * 当前机器人位置
         * @param enemyPositions
         * 敌方机器人位置
         * @param detectedEnemy
         * 检测到的地方目标ID
         * @param myWayPointID
         * 当前机器人所在路径点ID
         */
        int decideAimTarget(RobotPosition &mypos, std::vector<RobotPosition> &enemyPositions, std::vector<int> &detectedEnemy, int &myWayPointID);

        double decideAngleByEnemyPos(float _x, float _y, std::vector<RobotPosition> &enemyPositions);

        float getDistanceTHR();
        void setDistanceTHR(float thr);
        float getSeekTHR();
        void setSeekTHR(float thr);
    };
}
#endif
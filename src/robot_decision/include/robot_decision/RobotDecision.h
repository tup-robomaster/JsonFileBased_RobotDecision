#ifndef _RD_DECISION_H
#define _RD_DECISION_H

#include "./structs.h"

namespace robotdecision
{
    class RobotDecision
    {
    private:
        std::vector<WayPoint> wayPointMap;
        std::vector<Decision> decisions;

    private:
        int calculatePosition(RobotPosition &pos);
        std::vector<WayPoint> calculatePath(int startWapPointID, int endWapPointID);

    public:
        RobotDecision();
        ~RobotDecision();

        bool decodeWayPoints(char *filePath);
        bool decodeDecisions(char *filePath);
        
        int checkNowWayPoint(float x, float y);
        int checkNowWayPoint(RobotPosition pos);
        Decision decide(int wayPointID, int robot_mode, int _HP, int nowtime, std::vector<RobotPosition> friendPositions, std::vector<RobotPosition> enemyPositions);

        WayPoint getWayPointByID(int id);
        Decision getDecisionByID(int id);
    };
}
#endif
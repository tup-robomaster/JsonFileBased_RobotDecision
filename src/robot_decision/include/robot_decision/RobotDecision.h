#ifndef _RD_DECISION_H
#define _RD_DECISION_H

#include "./structs.h"

namespace rdsys
{
    /**
     * @brief 裁判系统信息处理类
     * 处理存储比赛信息
     */
    class GameHandler
    {
    private:
        std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> lastUpdateTime;
        int gameTime = -1;

    public:
        GameHandler();
        ~GameHandler();

        void update(int &gameTime);
    };
    /**
     * @brief 决策系统类
     * 提供机器人决策相关处理接口
     */
    class RobotDecisionSys
    {
    private:
        std::string _MAP_PATH;
        float _REAL_WIDTH;
        float _REAL_HEIGHT;
        float _STEP_DISTANCE;
        float _CAR_SEEK_FOV;

    private:
        bool IfShowUI = false;
        bool IfUIInited = false;

    public:
        /**
         * @brief 获取UI显示标志位
         * @return
         * 当前UI显示标志位
         */
        bool getIfShowUI();
        /**
         * @brief 设置UI显示标志位
         * @param ifShowUI
         * 目标UI显示标志位
         */
        void setIfShowUI(bool ifShowUI);

    private:
        std::vector<std::shared_ptr<WayPoint>> wayPointMap;
        std::vector<std::shared_ptr<Decision>> decisions;

        float _distance_THR = 0.;
        float _seek_THR = 5.0;

    private:
        std::map<int, std::vector<int>> connection_map;
        std::shared_ptr<GameHandler> myGameHandler = nullptr;

    public:
        /**
         * @brief 计算角度
         * @param x1
         * 点1 x坐标
         * @param y1
         * 点1 y坐标
         * @param x2
         * 点2 x坐标
         * @param y2
         * 点2 y坐标
         * @return
         * 角度值（弧度制）
         */
        double calculateAngle(double x1, double y1, double x2, double y2);
        /**
         * @brief 计算角度
         * @param p1
         * 点1
         * @param p2
         * 点2
         * @return
         * 角度值（弧度制）
         */
        double calculateAngle(cv::Point2d p1, cv::Point2d p2);

    private:
        /**
         * @brief 计算机器人当前所在路径点
         * @param pos
         * 当前机器人坐标信息
         */
        int calculatePosition(RobotPosition &pos);
        /**
         * @brief 通过角度获取终止点
         * @param x1
         * 点1 x坐标
         * @param y1
         * 点1 y坐标
         * @param theta
         * 角度（弧度制）
         * @param length
         * 两点距离
         * @return
         * 结束点
         */
        cv::Point2i createEndPointByTheta(double x1, double y1, double theta, int length);
        /**
         * @brief 通过角度获取终止点
         * @param start
         * 起点
         * @param theta
         * 角度（弧度制）
         * @param length
         * 两点距离
         * @return
         * 结束点
         */
        cv::Point2i createEndPointByTheta(cv::Point start, double theta, int length);
        /**
         * @brief 通过角度与起终点判断线上是否有障碍
         * @param start
         * 起点
         * @param theta
         * 角度（弧度制）
         * @param distance
         * 终点距离
         * @return
         * 是否有阻碍
         */
        bool checkBlock(cv::Point start, double theta, int distance);

    public:
        RobotDecisionSys(float &_distance_THR, float &_seek_THR, float &_REAL_WIDTH, float &_REAL_HEIGHT, std::string &_MAP_PATH, float &_STEP_DISTANCE, float &_CAR_SEEK_FOV);
        ~RobotDecisionSys();

        /**
         * @brief 解码路径点Json文件
         * @param filePath
         * 路径点Json文件地址
         * @return
         * 是否成功
         */
        bool decodeWayPoints(std::string &filePath);
        /**
         * @brief 解码决策Json文件
         * @param filePath
         * 决策Json文件地址
         * @return
         * 是否成功
         */
        bool decodeDecisions(std::string &filePath);

        /**
         * @brief 检查机器人当前所在路径点
         * @param x
         * 机器人坐标x
         * @param y
         * 机器人坐标y
         * @return
         * 路径点ID
         */
        int checkNowWayPoint(float x, float y);
        /**
         * @brief 检查机器人当前所在路径点
         * @param pos
         * 当前机器人坐标信息
         * @return
         * 路径点ID
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
         * @param nowtime
         * 当前比赛剩余时间
         * @param now_out_post_HP
         * 当前前哨站血量
         * @param now_base_HP
         * 当前基地血量
         * @param friendPositions
         * 友方机器人位置（真实坐标）
         * @param enemtPositions
         * 敌方机器人位置（真实坐标）
         * @param availableDecisionID
         * 输入输出符合条件的决策ID
         * @param id_pos_f
         * 友方位置（路径点）
         * @param id_pos_e
         * 敌方位置（路径点）
         * @return
         * 决策
         */
        std::shared_ptr<Decision> decide(int wayPointID, int robot_mode, int _HP, int nowtime, int now_out_post_HP, int now_base_HP, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, std::vector<int> &availableDecisionID, std::map<int, int> &id_pos_f, std::map<int, int> &id_pos_e);
        /**
         * @brief 计算路径路线
         * @param startWapPointID
         * 起点路径点ID
         * @param endWapPointID
         * 终点路径点ID
         * @return
         * 路径点集合
         */
        std::vector<std::shared_ptr<WayPoint>> calculatePath(int startWapPointID, int endWapPointID);
        /**
         * @brief 根据ID获取路径点
         * @param id
         * 路径点ID
         * @return
         * 路径点
         */
        std::shared_ptr<WayPoint> getWayPointByID(int id);
        /**
         * @brief 根据ID获取决策
         * @param id
         * 决策ID
         * @return
         * 决策
         */
        std::shared_ptr<Decision> getDecisionByID(int id);

        /**
         * @brief 决策打击目标
         * @param _IsBlue
         * 是否是蓝方
         * @param mypos
         * 当前机器人位置
         * @param enemyPositions
         * 敌方机器人位置
         * @param detectedEnemy
         * 检测到的敌方目标ID
         * @param enemyHP
         * 敌方血量(仅填入车辆血量)
         * @param distance_weight
         * 距离权重
         * @param hp_weight
         * 血量权重
         * @return 
         * 打击权重
         */
        std::map<int, float> decideAimTarget(bool _IsBlue, RobotPosition &mypos, std::vector<RobotPosition> &enemyPositions, std::vector<int> &enemyHP, float distance_weight_ratio, float hp_weight_ratio, bool enemyOutpostDown);
        /**
         * @brief 决策yaw轴角度，通用坐标系
         * @param _x
         * 当前机器人x轴坐标（真实坐标）
         * @param _y
         * 当前机器人y轴坐标（真实坐标）
         * @param enemyPositions
         * 敌方机器人位置（真实坐标）
         * @return
         * yaw轴角度
         */
        double decideAngleByEnemyPos(float _x, float _y, std::vector<RobotPosition> &enemyPositions);

        /**
         * @brief 获取距离阈值，用于计算路径点
         * @return
         * 距离阈值
         */
        float getDistanceTHR();
        /**
         * @brief 设置距离阈值，用于计算路径点
         * @param thr
         * 距离阈值
         */
        void setDistanceTHR(float thr);
        /**
         * @brief 获取距离阈值，用于索敌
         * @return
         * 距离阈值
         */
        float getSeekTHR();
        /**
         * @brief 设置距离阈值，用于索敌
         * @param thr
         * 距离阈值
         */
        void setSeekTHR(float thr);

    private:
        cv::Mat decisionMap;
        cv::Mat decisionMap_Gray;

    public:
        /**
         * @brief 更新路径状态图UI
         * @param activateDecisionID
         * 激活决策ID
         * @param availableDecisionID
         * 符合条件的决策ID
         * @param nowWayPoint
         * 当前所在路径点
         * @param yaw
         * 当前云台yaw轴角度（弧度制）
         * @param car_center
         * 当前车辆坐标（真实坐标）
         * @param car_orientation
         * 当前车辆朝向角度（弧度制）
         * @param aim_yaw
         * 目标云台yaw轴角度（弧度制）
         * @param friendPositions
         * 友方机器人位置（真实坐标）
         * @param enemtPositions
         * 敌方机器人位置（真实坐标）
         * @param id_pos_f
         * 友方位置（路径点）
         * @param id_pos_e
         * 敌方位置（路径点）
         * @param aimWayPoints
         * 决策路径点的集合
         */
        void UpdateDecisionMap(int &activateDecisionID, std::vector<int> &availableDecisionID, int &nowWayPoint, double yaw, cv::Point2f car_center, double car_orientation, double aim_yaw, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, std::map<int, int> &id_pos_f, std::map<int, int> &id_pos_e);

    private:
        /**
         * @brief 绘制路径点
         * @param img
         * 目标绘制图像
         * @param center
         * 路径点坐标（图像坐标）
         * @param id
         * 路径点ID
         * @param type
         * 路径点状态（0未激活-灰色，1符合条件-蓝色，2激活-绿色，3当前-黄色）
         */
        void drawWayPoint(cv::Mat &img, cv::Point2i center, int id, int type);
        /**
         * @brief 变换中心点
         * @param _x
         * 中心点x轴坐标（真实坐标）
         * @param _y
         * 中心点y轴坐标（真实坐标）
         * @param width
         * 场地宽度
         * @param height
         * 场地高度
         * @param img_cols
         * 图像宽度
         * @param img_rows
         * 图像高度
         * @return
         * 真实坐标对应的图像坐标
         */
        cv::Point2i transformPoint(float _x, float _y, float width, float height, int img_cols, int img_rows);
        /**
         * @brief 变换中心点
         * @param center
         * 中心点（真实坐标）
         * @param width
         * 场地宽度
         * @param height
         * 场地高度
         * @param img_cols
         * 图像宽度
         * @param img_rows
         * 图像高度
         * @return
         * 真实坐标对应的图像坐标
         */
        cv::Point2i transformPoint(cv::Point2f center, float width, float height, int img_cols, int img_rows);
        /**
         * @brief 绘制车辆
         * @param img
         * 目标绘制图像
         * @param center
         * 车辆位置（图像坐标）
         * @param car_orientation
         * 车辆朝向
         * @param yaw
         * yaw轴朝向
         */
        void drawCar(cv::Mat &img, cv::Point2i center, double &car_orientation, double &yaw, double &aim_yaw);
        /**
         * @brief 绘制敌方车辆
         * @param img
         * 目标绘制图像
         * @param center
         * 车辆位置（图像坐标）
         * @param type
         * 车辆所属(0：友方，1：敌方)
         */
        void drawOthCar(cv::Mat &img, cv::Point2i center, int &id, int type);
    };
}
#endif
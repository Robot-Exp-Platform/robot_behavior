#include <string>

class Robot
{
public:
    /// @brief get the version of the robot 获取机器人版本号
    /// @return A string representing the robot's version.
    virtual std::string version() = 0;

    /// @brief initialize the robot 初始化机器人
    virtual void init() = 0;

    /// @brief shutdown the robot 关闭机器人
    virtual void shutdown() = 0;

    /// @brief enable the robot 使能机器人
    virtual void enable() = 0;

    /// @brief disable the robot 去使能机器人
    virtual void disable() = 0;

    /// @brief reset the robot 复位机器人
    virtual void reset() = 0;

    /// @brief check if robot is moving 检查机器人是否在运动中
    /// @return bool: check if robot is moving 是否在运动状态
    virtual bool is_moving() = 0;

    /// @brief stop the current action 停止当前动作，不可恢复
    virtual void stop() = 0;

    /// @brief pause the robot 运动暂停
    virtual void pause() = 0;

    /// @brief resume the robot 运动恢复
    virtual void resume() = 0;

    /// @brief emergency stop the robot 紧急停止机器人
    virtual void emergency_stop() = 0;

    /// @brief clear emergency stop status 清除紧急停止状态
    virtual void clear_emergency_stop() = 0;
};

class Arm : Robot
{
};

class Pose
{
};

class Load
{
};
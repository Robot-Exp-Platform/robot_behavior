use crate::RobotResult;

/// # Robot Behavior
/// 机器人行为特征
pub trait RobotBehavior {
    type State;

    /// Get the robot version
    /// 获取机器人版本和驱动版本
    fn version() -> String;

    /// Initialize the robot
    /// 初始化机器人
    fn init(&mut self) -> RobotResult<()>;

    /// Shutdown the robot
    /// 关闭机器人
    fn shutdown(&mut self) -> RobotResult<()>;

    /// Enable the robot
    /// 使能机器人
    fn enable(&mut self) -> RobotResult<()>;

    /// Disable the robot
    /// 去使能机器人
    fn disable(&mut self) -> RobotResult<()>;

    /// reset the robot
    /// 复位机器人
    fn reset(&mut self) -> RobotResult<()>;

    /// Check if the robot is moving
    /// 检查机器人是否在运动
    fn is_moving(&mut self) -> bool;

    /// stop the current action
    /// 停止当前动作，不可恢复
    fn stop(&mut self) -> RobotResult<()>;

    /// pause the current action
    /// 暂停当前动作
    fn pause(&mut self) -> RobotResult<()>;

    /// Resume the current action
    /// 恢复当前动作
    fn resume(&mut self) -> RobotResult<()>;

    /// Emergency stop
    /// 紧急停止
    fn emergency_stop(&mut self) -> RobotResult<()>;

    /// Clear the emergency stop
    /// 清除紧急停止
    fn clear_emergency_stop(&mut self) -> RobotResult<()>;

    /// Get the robot state
    /// 获取机器人状态
    fn read_state(&mut self) -> RobotResult<Self::State>;
}

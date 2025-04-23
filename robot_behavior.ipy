class LoadState:
    m : float
    x : list[float]
    i : list[float]
    ...
    
class Pose:
    """
    # Pose
    The same position can be represented in different ways. Different description methods are uniformly expressed as the `Pose` object  
    同一个位置可以用不同的方式表示, 将不同的描述方式统一表述为 `Pose` 对象
    
    ## constructor 构造函数
    
    ```python
    pose = Pose.Euler([x, y, z], [roll, pitch, yaw])
    pose = Pose.Quat([x, y, z], [qx, qy, qz, qw])
    pose = Pose.Homo(homo_array: list)
    pose = Pose.AxisAngle([x, y, z], [axis_x, axis_y, axis_z], angle)
    pose = Pose.Position([x, y, z])
    ```
    
    ## methods 方法
    
    ```python
    (tran, rot) = pose.euler()
    (tran, rot) = pose.quat()
    (homo_array) = pose.homo()
    (tran, axis, angle) = pose.axis_angle()
    (tran) = pose.position()
    ```
    """
    
    def euler(self) -> tuple[list[float], list[float]]:
        """
        Convert the pose into the Euler Angle expression 将位姿转化为欧拉角表述  
        ```
        ([x, y, z], [roll, pitch, yaw])
        ```
        Returns the Euler angles as a tuple of two lists: translation and rotation.
        """
        ...
    
    def quat(self) -> tuple[list[float], list[float]]:
        """
        Convert the pose into the Quaternion expression 将位姿转化为四元数表述  
        ```
        ([x, y, z], [qx, qy, qz, qw])
        ```  
        Returns the quaternion as a tuple of two lists: translation and rotation. 
        """
        ...
        
    def homo(self) -> list[float]:
        """
        Convert the pose into the homogeneous representation 将位姿转化为齐次表示,这里并非其次变换矩阵，而是按列存储的列表  
        ```
        list[float]
        ```
        Returns the homogeneous representation as a list.
        """
        ...
        
    def axis_angle(self) -> tuple[list[float], list[float], float]:
        """
        Convert the pose into the Axis-Angle representation 将位姿转化为轴角表述  
        ```
        ([x, y, z], [axis_x, axis_y, axis_z], angle)
        ```
        Returns the axis-angle representation as a tuple of a list for translation, a list for the axis, and a float for the angle.
        """
        ...
        
    def position(self) -> list[float]:
        """
        Convert the pose into the Position representation 将位姿转化为位置表示  
        ```
        [x, y, z]
        ```
        Returns the position as a list.
        """
        ...

class RobotState:
    ...

class Robot:
    """
    # Robot
    
    ## methods(as `RobotBehavior`) 方法
    """
    
    def version(self) -> str:
        """get the version of the robot 获取机器人版本号
        Returns:
            str: the version of the robot 机器人版本号
        """
        ...

    def init(self) -> None:
        """initialize the robot 初始化机器人"""
        ...

    def shutdown(self) -> None:
        """shutdown the robot 关闭机器人"""
        ...

    def enable(self) -> None:
        """enable the robot 使能机器人"""
        ...

    def disable(self) -> None:
        """disable the robot 去使能机器人"""
        ...
        
    def reset(self) -> None:
        """reset the robot 复位机器人"""
        ...
    
    def is_moving(self) -> bool:
        """check if robot is moving 检查机器人是否在运动中
        Returns:
            bool: check if robot is moving 是否在运动状态
        """
        ...

    def stop(self) -> None:
        """stop the current action 停止当前动作，不可恢复"""
        ...
        
    def pause(self) -> None:
        """pause the robot 运动暂停"""
        ...

    def resume(self) -> None:
        """resume the robot 运动恢复"""
        ...

    def emergency_stop(self) -> None:
        """emergency stop the robot 紧急停止机器人"""
        ...

    def clear_emergency_stop(self) -> None:
        """clear emergency stop status 清除紧急停止状态"""
        ...
    
    def read_state(self) -> RobotState:
        """read the robot state 读取机器人状态
        Returns:
            RobotState: the robot state 机器人状态
        """
        ...
    ...

class MotionType:
    ...
    
class MotionType1(MotionType):
    ...
    
class MotionType2(MotionType):
    ...

class MotionType3(MotionType):
    ...
    
class MotionType4(MotionType):
    ...
    
class MotionType5(MotionType):
    ...
    
class MotionType6(MotionType):
    ...
    
class MotionType7(MotionType):
    ...

class ControlType:
    ...

class ControlType1(ControlType):
    ...

class ControlType2(ControlType):
    ...

class ControlType3(ControlType):
    ...

class ControlType4(ControlType):
    ...
    
class ControlType5(ControlType):
    ...
    
class ControlType6(ControlType):
    ...
    
class ControlType7(ControlType):
    ...
    
class ArmState:
    def joint(self) -> list[float] | None:
        ...
    def joint_vel(self) -> list[float] | None:
        ...
    def joint_acc(self) -> list[float] | None:
        ...
    def tau(self) -> list[float] | None:
        ...
    def pose_o_to_ee(self) -> Pose | None:
        """
        Get the pose of the end effector in the base frame 获取末端执行器在基坐标系下的位姿
        Returns:
            Pose: the pose of the end effector in the base frame 末端执行器在基坐标系下的位姿
        """
        ...
    def pose_f_to_ee(self) -> Pose | None:
        """
        Get the pose of the end effector in the flange frame 获取末端执行器在法兰坐标系下的位姿
        Returns:
            Pose: the pose of the end effector in the flange frame 末端执行器在法兰坐标系下的位姿
        """
        ...
    def pose_ee_to_kin(self) -> Pose | None:
        """
        Get the pose of the end effector in the kin frame 获取末端执行器在关节坐标系下的位姿
        Returns:
            Pose: the pose of the end effector in the kin frame 末端执行器在关节坐标系下的位姿
        """
        ...
    def cartesian_val(self) -> list[float] | None:
        """
        Get the cartesian velocity of the end effector 获取末端执行器在笛卡尔坐标系下的速度
        Returns:
            list[float]: the cartesian velocity of the end effector 末端执行器在笛卡尔坐标系下的速度
        """
        ...
    def load(self) -> LoadState | None:
        """
        Get the load state of the end effector 获取末端执行器的负载状态
        Returns:
            LoadState: the load state of the end effector 末端执行器的负载状态
        """
        ...
    ...

# TODO rewrite the docstring with `python` style 
class Arm(Robot):
    """
    # Robot Arm
    机械臂型机器人
    
    ## methods(as `ArmBehavior`) 方法
    
    ```
    type State;
    fn read_state(&mut self) -> RobotResult<Self::State>;
    fn state(&mut self) -> RobotResult<ArmState<N>>;
    fn set_load(&mut self, load: LoadState) -> RobotResult<()>;
    ```
    
    ## methods as `ArmPreplannedMotion`
    
    ```
    fn move_to(&mut self, target: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_to_async(&mut self, target: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_rel(&mut self, rel: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_rel_async(&mut self, rel: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_path(&mut self, path: Vec<MotionType<N>>, speed: f64) -> RobotResult<()>;
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()>;
    ```
    
    ## methods as `ArmPreplannedMotionExt`
    
    ```
    fn move_joint(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()>;
    fn move_joint_async(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()>;
    fn move_joint_rel(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()>;
    fn move_joint_rel_async(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()>;
    fn move_cartesian(&mut self, target: &Pose, speed: f64) -> RobotResult<()>;
    fn move_cartesian_async(&mut self, target: &Pose, speed: f64) -> RobotResult<()>;
    fn move_cartesian_rel(&mut self, target: &Pose, speed: f64) -> RobotResult<()>;
    fn move_cartesian_rel_async(&mut self, target: &Pose, speed: f64) -> RobotResult<()>;
    fn move_linear_with_euler(&mut self, tran: [f64; 3], rot: [f64; 3], speed: f64) -> RobotResult<()>;
    fn move_linear_with_euler_async(&mut self, tran: [f64; 3], rot: [f64; 3], speed: f64) -> RobotResult<()>;
    fn move_linear_with_homo(&mut self, target: &[f64; 16], speed: f64) -> RobotResult<()>;
    fn move_linear_with_homo_async(&mut self, target: &[f64; 16], speed: f64) -> RobotResult<()>;
    fn move_path_prepare(&mut self, _path: Vec<MotionType<N>>) -> RobotResult<()>;
    fn move_path_start(&mut self) -> RobotResult<()>;
    fn move_path_prepare_from_file(&mut self, path: &str) -> RobotResult<()>;
    fn move_path_from_file(&mut self, path: &str, speed: f64) -> RobotResult<()>;
    ```
    
    ## methods as `ArmStreamingHandle`
    
    ```
    type Handle: ArmStreamingHandle<N>;
    fn start_streaming(&mut self) -> RobotResult<Self::Handle>;
    fn end_streaming(&mut self) -> RobotResult<()>;

    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<N>>>>;
    fn control_to_target(&mut self) -> Arc<Mutex<Option<ControlType<N>>>>;
    ```
    
    ## methods as `ArmStreamingHandleExt`
    
    ```
    fn move_joint_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_joint_vel_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_joint_acc_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_cartesian_target(&mut self) -> Arc<Mutex<Option<Pose>>>;
    fn move_cartesian_vel_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>>;
    fn move_cartesian_euler_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>>;
    fn move_cartesian_quat_target(&mut self) -> Arc<Mutex<Option<na::Isometry3<f64>>>>;
    fn move_cartesian_homo_target(&mut self) -> Arc<Mutex<Option<[f64; 16]>>>;
    fn control_tau_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    ```
    
    ## methods as `ArmRealtimeControl`
    
    ```
    fn move_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> MotionType<N> + Send + 'static;
    fn control_with_closure<FC>(&mut self, closure: FC) -> RobotResult<()>
    where
        FC: Fn(ArmState<N>, Duration) -> ControlType<N> + Send + 'static;
    ```
    
    ## methods as `ArmRealtimeControlExt`
    
    ```
    fn move_joint_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> [f64; N] + Send + Sync + 'static;

    fn move_joint_vel_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> [f64; N] + Send + Sync + 'static;

    fn move_cartesian_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> Pose + Send + Sync + 'static;

    fn move_cartesian_vel_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> [f64; 6] + Send + Sync + 'static;

    fn move_cartesian_euler_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> ([f64; 3], [f64; 3]) + Send + Sync + 'static;

    fn move_cartesian_quat_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> na::Isometry3<f64> + Send + Sync + 'static;

    fn move_cartesian_homo_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> [f64; 16] + Send + Sync + 'static;
    ```
    """
    def state(self) -> ArmState:
        ...
    def set_load(self, load: LoadState) -> None:
        ...
    ...
    
class ArmPreplannedMotion:
    def move_to(self, target: MotionType, speed) -> None:
        ...
    def move_to_async(self, target: MotionType, speed: float) -> None:
        ...
    def move_rel(self, rel: MotionType, speed: float) -> None:
        ...
    def move_rel_async(self, rel: MotionType, speed: float) -> None:
        ...
    def move_path(self, path: list[MotionType], speed: float) -> None:
        ...
    def control_with(self, control: ControlType) -> None:
        ...
    ...
    
class ArmPreplannedMotionExt:
    def move_joint(self, joint: list[float], speed: float) -> None:
        """以关节角度方式移动机器人
        Args:
            joint: 关节角度列表（长度必须为 HANS_DOF）
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_joint_async(self, joint: list[float], speed: float) -> None:
        """以关节角度方式异步移动机器人
        Args:
            joint: 关节角度列表（长度必须为 HANS_DOF）
            speed: 运动速度（0.0~1.0）
        """
        ...

    def move_joint_rel(self, joint_rel: list[float], speed: float) -> None:
        """以关节角度方式相对移动机器人
        Args:
            joint_rel: 相对关节角度列表（长度必须为 HANS_DOF）
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_joint_rel_async(self, joint_rel: list[float], speed: float) -> None:
        """以关节角度方式异步相对移动机器人
        Args:
            joint_rel: 相对关节角度列表（长度必须为 HANS_DOF）
            speed: 运动速度（0.0~1.0）
        """
        ...
    
    def move_joint_path(self, joints: list[list[float]], speed: float) -> None:
        """以关节角度方式移动机器人
        Args:
            joints: 关节角度列表
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_cartesian(self, pose: Pose, speed: float) -> None:
        """以笛卡尔坐标系移动机器人
        Args:
            pose: 位姿描述
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_cartesian_async(self, pose: Pose, speed: float) -> None:
        """以笛卡尔坐标系异步移动机器人
        Args:
            pose: 位姿描述
            speed: 运动速度（0.0~1.0）
        """
        ...

    def move_cartesian_rel(self, pose_rel: Pose, speed: float) -> None:
        """以笛卡尔坐标系相对移动机器人
        Args:
            pose_rel: 位姿描述
            speed: 运动速度（0.0~1.0）
        """
        ...
    
    def move_cartesian_rel_async(self, pose_rel: Pose, speed: float) -> None:
        """以笛卡尔坐标系异步相对移动机器人
        Args:
            pose_rel: 位姿描述
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_cartesian_path(self, poses: list[Pose], speed: float) -> None:
        """以笛卡尔坐标系移动机器人
        Args:
            poses: 位姿描述
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_path_from_file(self, path: str, speed: float) -> None:
        """从文件中读取关节角度路径并执行
        Args:
            path: 文件路径
            speed: 运动速度（0.0~1.0）
        """
        ...
    ...

class ArmStreamingHandle:
    def move_to(self, target: MotionType) -> None:
        ...
    def last_motion(self) -> MotionType:
        """
        Get the last motion target 获取上一个运动目标
        Returns:
            MotionType: the last motion target 上一个运动目标
        """
        ...
    def control_with(self, control: ControlType) -> None:
        """
        Set the control target 设置控制目标
        Args:
            control: 控制目标
        """
        ...
    def last_control(self) -> ControlType:
        """
        Get the last control target 获取上一个控制目标
        Returns:
            ControlType: the last control target 上一个控制目标
        """
    ...
    
class ArmStreamingMotion:
    def start_streaming(self) -> ArmStreamingHandle:
        """start streaming the robot state 开始流式运动机器人状态"""
        ...
    def end_streaming(self) -> None:
        """end streaming the robot state 结束流式运动机器人状态"""
        ...
    ...
    
class ArmStreamingMotionExt:
    ...
    
class ArmRealtimeControl:
    def move_with_closure(self, closure: callable[[ArmState, float], MotionType]) -> None:
        """以闭包方式移动机器人
        Args:
            closure: 闭包函数，接受机器人状态和时间间隔，返回运动目标
        """
        ...
    def control_with_closure(self, closure: callable[[ArmState, float], ControlType]) -> None:
        """以闭包方式控制机器人
        Args:
            closure: 闭包函数，接受机器人状态和时间间隔，返回控制目标
        """
    ...
    
class ArmRealtimeControlExt:
    def move_joint_with_closure(self, closure: callable[[ArmState, float], list[float]]) -> None:
        """以闭包方式在位置环中移动机器人
        Args:
            closure: 闭包函数，接受机器人状态和时间间隔，返回关节角度列表（长度必须为 HANS_DOF）
        """
        ...
    def move_joint_vel_with_closure(self, closure: callable[[ArmState, float], list[float]]) -> None:
        """以闭包方式在速度环中移动机器人
        Args:
            closure: 闭包函数，接受机器人状态和时间间隔，返回关节角速度列表（长度必须为 HANS_DOF）
        """
    def move_cartesian_with_closure(self, closure: callable[[ArmState, float], Pose]) -> None:
        """以闭包方式在位置环中移动机器人
        Args:
            closure: 闭包函数，接受机器人状态和时间间隔，返回位姿描述
        """
        ...
    def move_cartesian_vel_with_closure(self, closure: callable[[ArmState, float], list[float]]) -> None:
        """以闭包方式在速度环中移动机器人
        Args:
            closure: 闭包函数，接受机器人状态和时间间隔，返回笛卡尔坐标系下的速度列表（长度必须为 6）
        """
    ...
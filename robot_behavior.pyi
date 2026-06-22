from typing import Callable, Protocol

Vec = list[float]
JointControlFn = Callable[["JointState", float], tuple[Vec, bool]]
ArmControlFn = Callable[["ArmState", float], tuple[Vec, bool]]
ArmPoseControlFn = Callable[["ArmState", float], tuple["Pose", bool]]


class LoadState:
    """End-effector payload model: mass, center of mass, and inertia."""

    m: float
    x: list[float]
    i: list[float]

    def __init__(self, m: float, x: list[float], i: list[float]) -> None: ...


class Desc:
    """Description of a map from one named space to another."""

    from_: str
    to: str

    def __init__(self, from_: str, to: str) -> None: ...


class Pose:
    """Rigid pose with interchangeable Euler, quaternion, matrix, and position forms."""

    @classmethod
    def Euler(cls, tran: list[float], rot: list[float]) -> "Pose": ...
    @classmethod
    def Quat(cls, tran: list[float], rot: list[float]) -> "Pose": ...
    @classmethod
    def Homo(cls, homo: list[float]) -> "Pose": ...
    @classmethod
    def AxisAngle(cls, tran: list[float], axis: list[float], angle: float) -> "Pose": ...
    @classmethod
    def Position(cls, tran: list[float]) -> "Pose": ...
    def euler(self) -> tuple[list[float], list[float]]: ...
    def quat(self) -> tuple[list[float], list[float]]: ...
    def homo(self) -> list[float]: ...
    def axis_angle(self) -> tuple[list[float], list[float], float]: ...
    def position(self) -> list[float]: ...


class MotionType:
    """Compatibility value object for legacy mixed motion APIs."""

    @classmethod
    def Joint(cls, joint: Vec) -> "MotionType": ...
    @classmethod
    def JointVel(cls, joint_vel: Vec) -> "MotionType": ...
    @classmethod
    def Cartesian(cls, pose: Pose) -> "MotionType": ...
    @classmethod
    def CartesianVel(cls, cartesian_vel: Vec) -> "MotionType": ...
    @classmethod
    def Position(cls, position: Vec) -> "MotionType": ...
    @classmethod
    def PositionVel(cls, position_vel: Vec) -> "MotionType": ...
    @classmethod
    def Stop(cls) -> "MotionType": ...


class JointSample:
    """A single joint-space sample. Fields may be absent if the driver cannot provide them."""

    q: Vec | None
    dq: Vec | None
    ddq: Vec | None
    tau: Vec | None
    dtau: Vec | None

    def __init__(
        self,
        q: Vec | None,
        dq: Vec | None,
        ddq: Vec | None,
        tau: Vec | None,
        dtau: Vec | None,
    ) -> None: ...


class SpatialSample:
    """A single spatial sample for flange, TCP, stiffness frame, or another spatial feature."""

    pose: Pose | None
    vel: Vec | None
    acc: Vec | None
    wrench: Vec | None

    def __init__(
        self,
        pose: Pose | None,
        vel: Vec | None,
        acc: Vec | None,
        wrench: Vec | None,
    ) -> None: ...


class JointState:
    """Measured, commanded, and desired joint-space state."""

    meas: JointSample
    cmd: JointSample
    des: JointSample

    def __init__(self, meas: JointSample, cmd: JointSample, des: JointSample) -> None: ...


class SpatialState:
    """Measured, commanded, and desired spatial state."""

    meas: SpatialSample
    cmd: SpatialSample
    des: SpatialSample

    def __init__(self, meas: SpatialSample, cmd: SpatialSample, des: SpatialSample) -> None: ...


class ArmState:
    """Composed arm state. Joint and flange are required; TCP, stiffness, and load are optional."""

    joint: JointState
    flange: SpatialState
    tcp: SpatialState | None
    stiffness: SpatialState | None
    load: LoadState | None

    def __init__(
        self,
        joint: JointState,
        flange: SpatialState,
        tcp: SpatialState | None,
        stiffness: SpatialState | None,
        load: LoadState | None,
    ) -> None: ...


class Robot(Protocol):
    """Common lifecycle and safety surface shared by robot drivers."""

    @staticmethod
    def version() -> str: ...
    def init(self) -> None: ...
    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def shutdown(self) -> None: ...
    def reset(self) -> None: ...
    def stop(self) -> None: ...
    def pause(self) -> None: ...
    def resume(self) -> None: ...
    def emergency_stop(self) -> None: ...
    def waiting_for_finish(self) -> None: ...
    def clear_emergency_stop(self) -> None: ...
    def is_moving(self) -> bool: ...


class Arm(Robot, Protocol):
    """Feature trait for manipulators exposing arm state, payload, and limits."""

    def state(self) -> ArmState: ...
    def set_load(self, load: LoadState) -> None: ...
    def get_joint(self) -> Vec: ...
    def get_endpoint(self) -> Pose: ...
    def get_joint_min(self) -> Vec: ...
    def get_joint_max(self) -> Vec: ...
    def get_joint_vel_bound(self) -> Vec: ...
    def get_joint_acc_bound(self) -> Vec: ...
    def get_joint_jerk_bound(self) -> Vec: ...
    def get_torque_bound(self) -> Vec: ...
    def get_torque_dot_bound(self) -> Vec: ...


class JointMotion(Protocol):
    """Joint-space point and trajectory motion."""

    def move_joint(self, target: Vec) -> None: ...
    def move_joint_sync(self, target: Vec) -> None: ...
    def move_joint_traj(self, traj: list[Vec]) -> None: ...


class FlangeMotion(Protocol):
    """Flange-space point motion."""

    def move_flange(self, target: Pose) -> None: ...
    def move_flange_sync(self, target: Pose) -> None: ...


class FlangeTrajectory(Protocol):
    """Flange-space trajectory motion."""

    def move_flange_traj(self, traj: list[Pose]) -> None: ...


class JointPositionControl(Protocol):
    def control_joint_position(self, closure: JointControlFn) -> None: ...


class JointVelocityControl(Protocol):
    def control_joint_velocity(self, closure: JointControlFn) -> None: ...


class JointTorqueControl(Protocol):
    def control_joint_torque(self, closure: JointControlFn) -> None: ...


class ArmTorqueControl(Protocol):
    def control_arm_torque(self, closure: ArmControlFn) -> None: ...


class CartesianVelocityControl(Protocol):
    def control_cartesian_velocity(self, closure: ArmControlFn) -> None: ...


class CartesianPoseControl(Protocol):
    def control_cartesian_pose(self, closure: ArmPoseControlFn) -> None: ...

use std::future::Future;

use serde::de::DeserializeOwned;

use crate::{Robot, RobotException, RobotResult};

/// Type-level mapping from a motion *space* to the *target* it carries for a
/// particular robot `R`.
///
/// This trait has no methods: it only fixes [`MotionSpace::Target`]. For
/// example `JointSpace` maps to the robot's joint vector, while `FlangeSpace`
/// and `TcpSpace` map to an `Iso3` pose.
pub trait MotionSpace<R: ?Sized> {
    /// The value type a command in this space carries.
    type Target;
}

/// Reach a single target in motion space `S`. **Implemented by drivers.**
///
/// One `impl MoveTo<S> for MyRobot` per space the driver supports. Higher
/// spaces typically delegate down a funnel (`Tcp → Flange → Joint → hardware`),
/// each hop applying its own linkage (tool offset, inverse kinematics, joint
/// limits).
pub trait MoveTo<S: MotionSpace<Self>>: Robot {
    /// Command the robot to reach `target`, returning when the motion has
    /// finished or failed.
    fn move_to(&mut self, target: S::Target) -> RobotResult<()>;

    /// Asynchronous target motion. The returned future is inert until the
    /// caller polls or awaits it.
    ///
    /// Drivers should implement this only when they have a real async backend.
    /// The default reports that async motion is not supported; it deliberately
    /// does not wrap the blocking [`MoveTo::move_to`] path.
    fn move_to_async(&mut self, _target: S::Target) -> impl Future<Output = RobotResult<()>> {
        async {
            Err(RobotException::UnprocessableInstructionError(
                "async target motion is not implemented for this driver".to_string(),
            ))
        }
    }
}

/// Follow a trajectory in motion space `S`. **Implemented by drivers.**
///
/// The three entry points form a funnel — `move_path` and `move_waypoints`
/// ultimately reduce to `move_traj`:
///
/// * [`MoveTraj::move_traj`] — the base primitive: a *dense*, already-sampled
///   trajectory dispatched directly to the hardware. **Required.**
/// * [`MoveTraj::move_path`] — a *continuous* path function `s ↦ target`,
///   sampled/planned into a dense trajectory and forwarded to `move_traj`.
/// * [`MoveTraj::move_waypoints`] — *sparse* waypoints, interpolated into a
///   dense trajectory and forwarded to `move_traj`.
///
/// Path sampling and waypoint interpolation are intentionally **left to the
/// driver/user**: the default `move_path` / `move_waypoints` simply report that
/// no planner is wired up. A driver opts in by overriding them and funnelling
/// into its own `move_traj`.
pub trait MoveTraj<S: MotionSpace<Self>>: Robot {
    /// Follow a dense, already-sampled trajectory. The base primitive every
    /// other trajectory entry point reduces to.
    fn move_traj(&mut self, traj: Vec<S::Target>) -> RobotResult<()>;
    fn move_traj_sync(&mut self, traj: Vec<S::Target>) -> RobotResult<()> {
        self.move_traj(traj)?;
        self.waiting_for_finish()
    }

    /// Follow a continuous path `path: s ↦ Some(target)` over `s ∈ [0, 1]`,
    /// returning `None` past the end. Defaults to "no planner wired up".
    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<S::Target>;
    fn move_path_sync<F>(&mut self, path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<S::Target>,
    {
        self.move_path(path)?;
        self.waiting_for_finish()
    }

    /// Follow a sequence of sparse waypoints. Defaults to "no planner wired up".
    fn move_waypoints(&mut self, _waypoints: Vec<S::Target>) -> RobotResult<()>;
    fn move_waypoints_sync(&mut self, waypoints: Vec<S::Target>) -> RobotResult<()> {
        self.move_waypoints(waypoints)?;
        self.waiting_for_finish()
    }
}

/// Ergonomic, space-parameterised entry point. **Blanket-implemented for every
/// robot** — users never implement this trait, they only call it:
///
/// ```ignore
/// arm.move_to::<JointSpace<7>>(target)?;
/// arm.move_traj::<FlangeSpace>(traj)?;
/// arm.move_path::<JointSpace<7>, _>(|s| Some(sample(s)))?;
/// arm.move_waypoints::<TcpSpace>(waypoints)?;
/// ```
///
/// Each method forwards to the corresponding [`MoveTo`] / [`MoveTraj`] impl the
/// driver provides for that space.
pub trait Motion: Sized {
    /// Reach `target` in space `S`. See [`MoveTo::move_to`].
    fn move_to<S>(&mut self, target: S::Target) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTo<S>,
    {
        <Self as MoveTo<S>>::move_to(self, target)
    }

    /// Reach `target` in space `S` asynchronously. See [`MoveTo::move_to_async`].
    fn move_to_async<S>(&mut self, target: S::Target) -> impl Future<Output = RobotResult<()>>
    where
        S: MotionSpace<Self>,
        Self: MoveTo<S>,
    {
        <Self as MoveTo<S>>::move_to_async(self, target)
    }

    #[deprecated(note = "move_to is blocking; use move_to_async for a Future-returning API")]
    #[allow(deprecated)]
    fn move_to_sync<S>(&mut self, target: S::Target) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTo<S>,
    {
        <Self as MoveTo<S>>::move_to(self, target)
    }

    /// Follow a dense trajectory in space `S`. See [`MoveTraj::move_traj`].
    fn move_traj<S>(&mut self, traj: Vec<S::Target>) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTraj<S>,
    {
        <Self as MoveTraj<S>>::move_traj(self, traj)
    }
    /// Follow a dense trajectory in space `S` and block. See [`MoveTraj::move_traj_sync`].
    fn move_traj_sync<S>(&mut self, traj: Vec<S::Target>) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTraj<S>,
    {
        <Self as MoveTraj<S>>::move_traj_sync(self, traj)
    }

    /// Follow a continuous path in space `S`. See [`MoveTraj::move_path`].
    fn move_path<S, F>(&mut self, path: F) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTraj<S>,
        F: Fn(f64) -> Option<S::Target>,
    {
        <Self as MoveTraj<S>>::move_path(self, path)
    }
    /// Follow a continuous path in space `S` and block. See [`MoveTraj::move_path_sync`].
    fn move_path_sync<S, F>(&mut self, path: F) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTraj<S>,
        F: Fn(f64) -> Option<S::Target>,
    {
        <Self as MoveTraj<S>>::move_path_sync(self, path)
    }

    /// Follow sparse waypoints in space `S`. See [`MoveTraj::move_waypoints`].
    fn move_waypoints<S>(&mut self, waypoints: Vec<S::Target>) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTraj<S>,
    {
        <Self as MoveTraj<S>>::move_waypoints(self, waypoints)
    }
    fn move_waypoints_sync<S>(&mut self, waypoints: Vec<S::Target>) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTraj<S>,
    {
        <Self as MoveTraj<S>>::move_waypoints_sync(self, waypoints)
    }
}
impl<R> Motion for R {}

pub trait MotionFile {
    fn move_traj_from_file<S>(&mut self, path: &str) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTraj<S>,
        S::Target: DeserializeOwned,
    {
        let file = std::fs::File::open(path)?;
        let traj: Vec<S::Target> = serde_json::from_reader(file)?;
        <Self as MoveTraj<S>>::move_traj(self, traj)
    }
    fn move_waypoints_from_file<S>(&mut self, path: &str) -> RobotResult<()>
    where
        S: MotionSpace<Self>,
        Self: MoveTraj<S>,
        S::Target: DeserializeOwned,
    {
        let file = std::fs::File::open(path)?;
        let wps: Vec<S::Target> = serde_json::from_reader(file)?;
        <Self as MoveTraj<S>>::move_waypoints(self, wps)
    }
}
impl<R> MotionFile for R {}

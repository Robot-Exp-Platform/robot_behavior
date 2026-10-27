//! One robot model shared by kinematics, simulation, visualization and limit
//! checks.
//!
//! [`RobotModel`] is the kinematic tree read from URDF: links with inertia and
//! geometry, joints with their origin, axis and limits. A [`Chain`] cuts one
//! base-to-tip path out of the tree for forward kinematics and per-joint
//! bounds, and its geometric [Jacobian](Chain::jacobian). Poses use
//! [`roplat_msgs::Pose`]; limits convert to [`roplat_msgs::JointLimits`].
//! [`RobotModel::link_frames`] places every link of the tree at once, for
//! visualization; [`RobotModel::inverse_dynamics`] gives the joint efforts
//! for a motion, gravity compensation included.
//!
//! Mesh file names are kept as written (`package://…`, relative paths);
//! resolving them is the consumer's job, as for
//! `roplat_behavior::SimulationModel` paths. DH parameters remain a separate
//! source in `roplat_kinematics`, whose `ArmKineCache::from_chain` also accepts
//! a [`Chain`].

use std::path::Path;

use nalgebra as na;
use roplat_msgs::{JointLimits, Pose, Vector3};

#[derive(Debug, thiserror::Error)]
pub enum DescriptionError {
    #[error("URDF: {0}")]
    Urdf(String),
    #[error("unknown link `{0}`")]
    UnknownLink(String),
    #[error("joint `{joint}` refers to unknown link `{link}`")]
    DanglingJoint { joint: String, link: String },
    #[error("link `{0}` has more than one parent joint")]
    MultipleParents(String),
    #[error("expected exactly one root link, found {0}")]
    RootCount(usize),
    #[error("`{base}` is not an ancestor of `{tip}`")]
    NotAncestor { base: String, tip: String },
    #[error("chain has {expected} movable joints, got {got} positions")]
    DofMismatch { expected: usize, got: usize },
    /// Some motion of the tree has no inertia (a movable joint carries no
    /// mass), so the mass matrix cannot be inverted.
    #[error("the mass matrix is singular: a movable joint moves no inertia")]
    SingularMassMatrix,
}

impl From<urdf_rs::UrdfError> for DescriptionError {
    fn from(e: urdf_rs::UrdfError) -> Self {
        Self::Urdf(e.to_string())
    }
}

/// Mass properties at `origin`, inertia tensor as `[ixx, ixy, ixz, iyy, iyz, izz]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertial {
    pub origin: Pose,
    pub mass: f64,
    pub inertia: [f64; 6],
}

#[derive(Clone, Debug, PartialEq)]
pub enum Geometry {
    Box { size: [f64; 3] },
    Cylinder { radius: f64, length: f64 },
    Capsule { radius: f64, length: f64 },
    Sphere { radius: f64 },
    Mesh { filename: String, scale: [f64; 3] },
}

/// A visual or collision shape placed in its link frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Shape {
    pub origin: Pose,
    pub geometry: Geometry,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Link {
    pub name: String,
    /// `None` when the description gives no positive mass.
    pub inertial: Option<Inertial>,
    pub visuals: Vec<Shape>,
    pub collisions: Vec<Shape>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
    Floating,
    Planar,
    Spherical,
}

/// Position bounds (rad or m) and velocity/effort magnitudes. A continuous
/// joint has infinite position bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimit {
    pub lower: f64,
    pub upper: f64,
    pub velocity: f64,
    pub effort: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    /// Index into [`RobotModel::links`].
    pub parent: usize,
    /// Index into [`RobotModel::links`].
    pub child: usize,
    /// Child frame in the parent frame at zero position.
    pub origin: Pose,
    /// Unit axis in the joint frame.
    pub axis: Vector3,
    /// Present for revolute, continuous and prismatic joints.
    pub limit: Option<JointLimit>,
}

impl Joint {
    /// Joints [`Chain`] drives with one scalar position each.
    pub fn is_movable(&self) -> bool {
        matches!(
            self.kind,
            JointKind::Revolute | JointKind::Continuous | JointKind::Prismatic
        )
    }

    /// Child frame in the parent frame at position `q`; `q` is ignored for
    /// joints that are not [movable](Self::is_movable).
    pub fn transform(&self, q: f64) -> na::Isometry3<f64> {
        let origin = self.origin.quat();
        let axis = na::Unit::new_normalize(na::Vector3::new(self.axis.x, self.axis.y, self.axis.z));
        let motion = match self.kind {
            JointKind::Revolute | JointKind::Continuous => na::Isometry3::from_parts(
                na::Translation3::identity(),
                na::UnitQuaternion::from_axis_angle(&axis, q),
            ),
            JointKind::Prismatic => na::Isometry3::translation(axis.x * q, axis.y * q, axis.z * q),
            _ => na::Isometry3::identity(),
        };
        origin * motion
    }
}

/// A kinematic tree with one root link.
#[derive(Clone, Debug, PartialEq)]
pub struct RobotModel {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
    root: usize,
    /// `parent_joint[link]` is the joint whose child is `link`.
    parent_joint: Vec<Option<usize>>,
    /// Joint indices with every parent before its children.
    order: Vec<usize>,
}

impl RobotModel {
    pub fn from_urdf_str(xml: &str) -> Result<Self, DescriptionError> {
        Self::from_urdf(&urdf_rs::read_from_string(xml)?)
    }

    pub fn from_urdf_file(path: impl AsRef<Path>) -> Result<Self, DescriptionError> {
        Self::from_urdf(&urdf_rs::read_file(path)?)
    }

    pub fn from_urdf(robot: &urdf_rs::Robot) -> Result<Self, DescriptionError> {
        let links: Vec<Link> = robot.links.iter().map(convert_link).collect();
        let index = |joint: &str, link: &str| {
            links.iter().position(|l| l.name == link).ok_or_else(|| {
                DescriptionError::DanglingJoint { joint: joint.into(), link: link.into() }
            })
        };

        let mut joints = Vec::with_capacity(robot.joints.len());
        let mut parent_joint = vec![None; links.len()];
        for (i, j) in robot.joints.iter().enumerate() {
            let parent = index(&j.name, &j.parent.link)?;
            let child = index(&j.name, &j.child.link)?;
            if parent_joint[child].replace(i).is_some() {
                return Err(DescriptionError::MultipleParents(links[child].name.clone()));
            }
            joints.push(convert_joint(j, parent, child));
        }

        let roots: Vec<usize> = (0..links.len())
            .filter(|&l| parent_joint[l].is_none())
            .collect();
        let [root] = roots[..] else {
            return Err(DescriptionError::RootCount(roots.len()));
        };

        // Breadth-first from the root. With one parent per link and a single
        // root every joint is reached exactly once.
        let mut order = Vec::with_capacity(joints.len());
        let mut frontier = vec![root];
        while !frontier.is_empty() {
            let mut next = Vec::new();
            for (i, joint) in joints.iter().enumerate() {
                if frontier.contains(&joint.parent) {
                    order.push(i);
                    next.push(joint.child);
                }
            }
            frontier = next;
        }
        Ok(Self {
            name: robot.name.clone(),
            links,
            joints,
            root,
            parent_joint,
            order,
        })
    }

    pub fn root(&self) -> &Link {
        &self.links[self.root]
    }

    pub fn link(&self, name: &str) -> Option<&Link> {
        self.links.iter().find(|l| l.name == name)
    }

    pub fn joint(&self, name: &str) -> Option<&Joint> {
        self.joints.iter().find(|j| j.name == name)
    }

    /// Every link frame in the root frame, indexed like [`links`](Self::links).
    ///
    /// `position` gives each [movable](Joint::is_movable) joint's position;
    /// it is not called for other joints. Branches (grippers, multi-arm
    /// trees) are all placed.
    pub fn link_frames(&self, mut position: impl FnMut(&Joint) -> f64) -> Vec<na::Isometry3<f64>> {
        let mut frames = vec![na::Isometry3::identity(); self.links.len()];
        for &j in &self.order {
            let joint = &self.joints[j];
            let q = if joint.is_movable() {
                position(joint)
            } else {
                0.0
            };
            frames[joint.child] = frames[joint.parent] * joint.transform(q);
        }
        frames
    }

    /// Joint forces and torques that give each movable joint the
    /// acceleration `qdd` at position `q` and velocity `qd`, with the root
    /// link fixed: the recursive Newton–Euler algorithm over the whole tree,
    /// branches included. `state` gives `(q, qd, qdd)` per movable joint;
    /// `gravity` is the gravitational acceleration in the root frame, e.g.
    /// `[0.0, 0.0, -9.81]`. With zero `qd` and `qdd` the result is the
    /// gravity torque.
    ///
    /// The result is indexed like [`joints`](Self::joints): torque (N·m) for
    /// revolute and continuous joints, force (N) for prismatic ones, zero for
    /// the rest. Links without [`Inertial`] carry nothing; this includes a
    /// link that declares zero mass with a nonzero inertia tensor (Panda's
    /// `panda_link8` in Bullet's data), which Bullet keeps as rotational
    /// inertia. Otherwise the result agrees with Bullet's
    /// `calculateInverseDynamics` to its single precision.
    pub fn inverse_dynamics(
        &self,
        mut state: impl FnMut(&Joint) -> (f64, f64, f64),
        gravity: [f64; 3],
    ) -> Vec<f64> {
        type V = na::Vector3<f64>;
        let n = self.links.len();
        let mut frames = vec![na::Isometry3::identity(); n];
        let mut omega = vec![V::zeros(); n];
        let mut alpha = vec![V::zeros(); n];
        // Acceleration of each link origin; the root accelerating against
        // gravity stands in for gravity acting on every link.
        let mut accel = vec![V::zeros(); n];
        accel[self.root] = -V::from(gravity);
        let mut axes = vec![V::zeros(); self.joints.len()];

        for &j in &self.order {
            let joint = &self.joints[j];
            let (p, c) = (joint.parent, joint.child);
            let (q, qd, qdd) = if joint.is_movable() {
                state(joint)
            } else {
                (0.0, 0.0, 0.0)
            };
            frames[c] = frames[p] * joint.transform(q);
            let z =
                frames[c].rotation * V::new(joint.axis.x, joint.axis.y, joint.axis.z).normalize();
            axes[j] = z;
            let r = frames[c].translation.vector - frames[p].translation.vector;
            let carried = accel[p] + alpha[p].cross(&r) + omega[p].cross(&omega[p].cross(&r));
            match joint.kind {
                JointKind::Revolute | JointKind::Continuous => {
                    omega[c] = omega[p] + z * qd;
                    alpha[c] = alpha[p] + z * qdd + omega[p].cross(&(z * qd));
                    accel[c] = carried;
                }
                JointKind::Prismatic => {
                    omega[c] = omega[p];
                    alpha[c] = alpha[p];
                    accel[c] = carried + omega[p].cross(&(z * qd)) * 2.0 + z * qdd;
                }
                _ => {
                    omega[c] = omega[p];
                    alpha[c] = alpha[p];
                    accel[c] = carried;
                }
            }
        }

        // Net force on each link and moment about its origin, in the root frame.
        let mut force = vec![V::zeros(); n];
        let mut moment = vec![V::zeros(); n];
        for (i, link) in self.links.iter().enumerate() {
            let Some(inertial) = link.inertial else {
                continue;
            };
            let com = frames[i] * inertial.origin.quat();
            let rc = com.translation.vector - frames[i].translation.vector;
            let a_com = accel[i] + alpha[i].cross(&rc) + omega[i].cross(&omega[i].cross(&rc));
            let [ixx, ixy, ixz, iyy, iyz, izz] = inertial.inertia;
            let local = na::Matrix3::new(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz);
            let rotation = com.rotation.to_rotation_matrix();
            let world = rotation.matrix() * local * rotation.matrix().transpose();
            force[i] = a_com * inertial.mass;
            moment[i] =
                world * alpha[i] + omega[i].cross(&(world * omega[i])) + rc.cross(&force[i]);
        }

        let mut efforts = vec![0.0; self.joints.len()];
        for &j in self.order.iter().rev() {
            let joint = &self.joints[j];
            let (p, c) = (joint.parent, joint.child);
            efforts[j] = match joint.kind {
                JointKind::Revolute | JointKind::Continuous => axes[j].dot(&moment[c]),
                JointKind::Prismatic => axes[j].dot(&force[c]),
                _ => 0.0,
            };
            let r = frames[c].translation.vector - frames[p].translation.vector;
            let (f, m) = (force[c], moment[c]);
            force[p] += f;
            moment[p] += m + r.cross(&f);
        }
        efforts
    }

    /// Indices into [`joints`](Self::joints) of the movable joints, in file
    /// order: the coordinate order of [`mass_matrix`](Self::mass_matrix) and
    /// [`forward_dynamics`](Self::forward_dynamics).
    pub fn movable_joints(&self) -> Vec<usize> {
        (0..self.joints.len())
            .filter(|&j| self.joints[j].is_movable())
            .collect()
    }

    /// The joint-space mass matrix at positions `q` (one per movable joint,
    /// in [`movable_joints`](Self::movable_joints) order), fixed root: column
    /// `i` is the effort for a unit acceleration of joint `i` with no velocity
    /// and no gravity. Same inertia conventions as
    /// [`inverse_dynamics`](Self::inverse_dynamics).
    pub fn mass_matrix(&self, q: &[f64]) -> Result<na::DMatrix<f64>, DescriptionError> {
        let movable = self.movable_joints();
        if q.len() != movable.len() {
            return Err(DescriptionError::DofMismatch { expected: movable.len(), got: q.len() });
        }
        let slot = self.coordinate_slots(&movable);
        let n = movable.len();
        let mut m = na::DMatrix::zeros(n, n);
        for i in 0..n {
            let efforts = self.inverse_dynamics(
                |joint| {
                    let k = slot(joint);
                    (q[k], 0.0, if k == i { 1.0 } else { 0.0 })
                },
                [0.0; 3],
            );
            for (row, &j) in movable.iter().enumerate() {
                m[(row, i)] = efforts[j];
            }
        }
        Ok(m)
    }

    /// Joint accelerations produced by efforts `tau` at positions `q` and
    /// velocities `qd` (all in [`movable_joints`](Self::movable_joints)
    /// order) under `gravity`, fixed root: `M(q)⁻¹ (τ − h(q, q̇))`, where `h`
    /// is the inverse dynamics at zero acceleration.
    pub fn forward_dynamics(
        &self,
        q: &[f64],
        qd: &[f64],
        tau: &[f64],
        gravity: [f64; 3],
    ) -> Result<Vec<f64>, DescriptionError> {
        let movable = self.movable_joints();
        for len in [qd.len(), tau.len()] {
            if len != movable.len() {
                return Err(DescriptionError::DofMismatch { expected: movable.len(), got: len });
            }
        }
        let m = self.mass_matrix(q)?;
        let slot = self.coordinate_slots(&movable);
        let bias = self.inverse_dynamics(
            |joint| {
                let k = slot(joint);
                (q[k], qd[k], 0.0)
            },
            gravity,
        );
        let rhs = na::DVector::from_iterator(
            movable.len(),
            movable.iter().zip(tau).map(|(&j, t)| t - bias[j]),
        );
        let qdd = m
            .cholesky()
            .ok_or(DescriptionError::SingularMassMatrix)?
            .solve(&rhs);
        Ok(qdd.as_slice().to_vec())
    }

    /// Maps a movable joint to its coordinate index; joint names are unique
    /// in a URDF.
    fn coordinate_slots(&self, movable: &[usize]) -> impl Fn(&Joint) -> usize {
        let slots: std::collections::HashMap<&str, usize> = movable
            .iter()
            .enumerate()
            .map(|(k, &j)| (self.joints[j].name.as_str(), k))
            .collect();
        move |joint: &Joint| slots[joint.name.as_str()]
    }

    /// The joints from `base` down to `tip`, in order.
    pub fn chain(&self, base: &str, tip: &str) -> Result<Chain, DescriptionError> {
        let find = |name: &str| {
            self.links
                .iter()
                .position(|l| l.name == name)
                .ok_or_else(|| DescriptionError::UnknownLink(name.into()))
        };
        let (base_index, mut link) = (find(base)?, find(tip)?);
        let mut joints = Vec::new();
        while link != base_index {
            let Some(j) = self.parent_joint[link] else {
                return Err(DescriptionError::NotAncestor { base: base.into(), tip: tip.into() });
            };
            joints.push(self.joints[j].clone());
            link = self.joints[j].parent;
        }
        joints.reverse();
        Ok(Chain { base: base.into(), tip: tip.into(), joints })
    }
}

/// One base-to-tip path, fixed joints included; positions address only the
/// movable ones, in order.
#[derive(Clone, Debug, PartialEq)]
pub struct Chain {
    pub base: String,
    pub tip: String,
    pub joints: Vec<Joint>,
}

impl Chain {
    pub fn movable(&self) -> impl Iterator<Item = &Joint> {
        self.joints.iter().filter(|j| j.is_movable())
    }

    pub fn dof(&self) -> usize {
        self.movable().count()
    }

    pub fn joint_names(&self) -> Vec<&str> {
        self.movable().map(|j| j.name.as_str()).collect()
    }

    /// Tip frame in the base frame.
    pub fn forward_kinematics(&self, q: &[f64]) -> Result<Pose, DescriptionError> {
        self.forward_kinematics_iso(q).map(Pose::from_isometry)
    }

    pub fn forward_kinematics_iso(
        &self,
        q: &[f64],
    ) -> Result<na::Isometry3<f64>, DescriptionError> {
        let expected = self.dof();
        if q.len() != expected {
            return Err(DescriptionError::DofMismatch { expected, got: q.len() });
        }
        let mut q = q.iter();
        Ok(self
            .joints
            .iter()
            .fold(na::Isometry3::identity(), |acc, joint| {
                let position = if joint.is_movable() {
                    *q.next().unwrap()
                } else {
                    0.0
                };
                acc * joint.transform(position)
            }))
    }

    /// The `6 x dof` geometric Jacobian of the tip in the base frame, linear
    /// rows first: column `i` maps the velocity of the `i`-th movable joint
    /// to the tip's `[vx, vy, vz, wx, wy, wz]`.
    pub fn jacobian(&self, q: &[f64]) -> Result<na::Matrix6xX<f64>, DescriptionError> {
        let expected = self.dof();
        if q.len() != expected {
            return Err(DescriptionError::DofMismatch { expected, got: q.len() });
        }
        // Joint axes and origins in the base frame, then the tip.
        let mut axes = Vec::with_capacity(expected);
        let mut frame = na::Isometry3::identity();
        let mut q = q.iter();
        for joint in &self.joints {
            let position = if joint.is_movable() {
                // The joint frame before its motion; the axis is given in it.
                let joint_frame = frame * joint.origin.quat();
                let axis = joint_frame.rotation
                    * na::Vector3::new(joint.axis.x, joint.axis.y, joint.axis.z).normalize();
                axes.push((joint.kind, axis, joint_frame.translation.vector));
                *q.next().unwrap()
            } else {
                0.0
            };
            frame *= joint.transform(position);
        }
        let tip = frame.translation.vector;

        let mut jacobian = na::Matrix6xX::zeros(expected);
        for (i, (kind, axis, origin)) in axes.into_iter().enumerate() {
            let (linear, angular) = match kind {
                JointKind::Prismatic => (axis, na::Vector3::zeros()),
                _ => (axis.cross(&(tip - origin)), axis),
            };
            jacobian.fixed_view_mut::<3, 1>(0, i).copy_from(&linear);
            jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&angular);
        }
        Ok(jacobian)
    }

    /// Position, velocity and effort bounds of the movable joints. The
    /// description has no acceleration bounds; `ddq_max` is infinite.
    pub fn joint_limits<const N: usize>(&self) -> Result<JointLimits<N>, DescriptionError> {
        let expected = self.dof();
        if expected != N {
            return Err(DescriptionError::DofMismatch { expected, got: N });
        }
        let mut limits = JointLimits {
            q_min: [f64::NEG_INFINITY; N],
            q_max: [f64::INFINITY; N],
            dq_max: [f64::INFINITY; N],
            ddq_max: [f64::INFINITY; N],
            tau_max: [f64::INFINITY; N],
        };
        for (i, joint) in self.movable().enumerate() {
            if let Some(l) = joint.limit {
                limits.q_min[i] = l.lower;
                limits.q_max[i] = l.upper;
                limits.dq_max[i] = l.velocity;
                limits.tau_max[i] = l.effort;
            }
        }
        Ok(limits)
    }

    /// Names of the joints whose position in `q` is outside its bounds.
    pub fn violated_position_limits(&self, q: &[f64]) -> Result<Vec<&str>, DescriptionError> {
        let expected = self.dof();
        if q.len() != expected {
            return Err(DescriptionError::DofMismatch { expected, got: q.len() });
        }
        Ok(self
            .movable()
            .zip(q)
            .filter(|(j, q)| j.limit.is_some_and(|l| **q < l.lower || **q > l.upper))
            .map(|(j, _)| j.name.as_str())
            .collect())
    }
}

fn convert_pose(p: &urdf_rs::Pose) -> Pose {
    let [x, y, z] = p.xyz.0;
    let [roll, pitch, yaw] = p.rpy.0;
    Pose::from_isometry(na::Isometry3::from_parts(
        na::Translation3::new(x, y, z),
        na::UnitQuaternion::from_euler_angles(roll, pitch, yaw),
    ))
}

fn convert_geometry(g: &urdf_rs::Geometry) -> Geometry {
    match g {
        urdf_rs::Geometry::Box { size } => Geometry::Box { size: size.0 },
        urdf_rs::Geometry::Cylinder { radius, length } => {
            Geometry::Cylinder { radius: *radius, length: *length }
        }
        urdf_rs::Geometry::Capsule { radius, length } => {
            Geometry::Capsule { radius: *radius, length: *length }
        }
        urdf_rs::Geometry::Sphere { radius } => Geometry::Sphere { radius: *radius },
        urdf_rs::Geometry::Mesh { filename, scale } => Geometry::Mesh {
            filename: filename.clone(),
            scale: scale.as_ref().map_or([1.0; 3], |s| s.0),
        },
    }
}

fn convert_link(l: &urdf_rs::Link) -> Link {
    let i = &l.inertial;
    let inertial = (i.mass.value > 0.0).then(|| Inertial {
        origin: convert_pose(&i.origin),
        mass: i.mass.value,
        inertia: [
            i.inertia.ixx,
            i.inertia.ixy,
            i.inertia.ixz,
            i.inertia.iyy,
            i.inertia.iyz,
            i.inertia.izz,
        ],
    });
    Link {
        name: l.name.clone(),
        inertial,
        visuals: l
            .visual
            .iter()
            .map(|v| Shape {
                origin: convert_pose(&v.origin),
                geometry: convert_geometry(&v.geometry),
            })
            .collect(),
        collisions: l
            .collision
            .iter()
            .map(|c| Shape {
                origin: convert_pose(&c.origin),
                geometry: convert_geometry(&c.geometry),
            })
            .collect(),
    }
}

fn convert_joint(j: &urdf_rs::Joint, parent: usize, child: usize) -> Joint {
    use urdf_rs::JointType as T;
    let kind = match j.joint_type {
        T::Fixed => JointKind::Fixed,
        T::Revolute => JointKind::Revolute,
        T::Continuous => JointKind::Continuous,
        T::Prismatic => JointKind::Prismatic,
        T::Floating => JointKind::Floating,
        T::Planar => JointKind::Planar,
        T::Spherical => JointKind::Spherical,
    };
    let l = &j.limit;
    let limit = match kind {
        JointKind::Revolute | JointKind::Prismatic => Some(JointLimit {
            lower: l.lower,
            upper: l.upper,
            velocity: l.velocity,
            effort: l.effort,
        }),
        JointKind::Continuous => Some(JointLimit {
            lower: f64::NEG_INFINITY,
            upper: f64::INFINITY,
            velocity: l.velocity,
            effort: l.effort,
        }),
        _ => None,
    };
    let [x, y, z] = j.axis.xyz.0;
    Joint {
        name: j.name.clone(),
        kind,
        parent,
        child,
        origin: convert_pose(&j.origin),
        axis: Vector3 { x, y, z },
        limit,
    }
}

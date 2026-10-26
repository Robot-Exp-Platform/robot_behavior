//! One robot model shared by kinematics, simulation, visualization and limit
//! checks.
//!
//! [`RobotModel`] is the kinematic tree read from URDF: links with inertia and
//! geometry, joints with their origin, axis and limits. A [`Chain`] cuts one
//! base-to-tip path out of the tree for forward kinematics and per-joint
//! bounds. Poses use [`roplat_msgs::Pose`]; limits convert to
//! [`roplat_msgs::JointLimits`].
//!
//! Mesh file names are kept as written (`package://…`, relative paths);
//! resolving them is the consumer's job, as for
//! `roplat_behavior::SimulationModel` paths. DH parameters remain a separate
//! source in `roplat_kinematics`.

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
        Ok(Self { name: robot.name.clone(), links, joints, root, parent_joint })
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

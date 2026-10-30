//! Hand-eye calibration: where a camera is relative to the arm, from paired
//! observations of the arm and a calibration target.
//!
//! Each [`HandEyeSample`] pairs the flange pose in the robot's base frame
//! (forward kinematics at a station) with the target pose in the camera's
//! optical frame (a marker or checkerboard detector at the same instant).
//! [`calibrate_hand_eye`] solves `A X = X B` over every pair of stations:
//! the rotation by the Park–Martin method (a least-squares fit of the
//! relative motions' rotation vectors), then the translation by linear least
//! squares. It also estimates where the target is and how consistently the
//! samples place it, which is the figure to check before trusting a result.
//!
//! - [`HandEye::EyeInHand`]: the camera rides on the flange and the target is
//!   fixed in the cell; the result is the camera in the flange frame.
//! - [`HandEye::EyeToHand`]: the camera is fixed in the cell and the target
//!   rides on the flange; the result is the camera in the base frame.
//!
//! The result enters a [`FrameTree`](crate::FrameTree) as a fixed edge:
//!
//! ```
//! # use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
//! use roplat_frames::calibration::{HandEye, HandEyeSample, calibrate_hand_eye};
//! use roplat_frames::FrameTree;
//!
//! # let camera = Isometry3::from_parts(Translation3::new(0.05, 0.0, 0.04), UnitQuaternion::from_euler_angles(0.0, 0.0, 1.57));
//! # let target = Isometry3::translation(0.6, 0.1, 0.0);
//! # let stations = [(0.1, 0.0, 0.0), (0.0, 0.3, 0.1), (0.2, -0.2, 0.4), (-0.3, 0.1, 0.2)];
//! # let samples: Vec<HandEyeSample> = stations.iter().map(|&(r, p, y)| {
//! #     let flange = Isometry3::from_parts(Translation3::new(0.4, 0.0, 0.5), UnitQuaternion::from_euler_angles(r, p, y));
//! #     HandEyeSample { flange, target: (flange * camera).inverse() * target }
//! # }).collect();
//! let calibration = calibrate_hand_eye(&samples, HandEye::EyeInHand)?;
//! assert!(calibration.translation_rms < 1e-3, "{calibration:?}");
//! let tree = FrameTree::builder()
//!     .fixed("base", "flange", roplat_msgs::Pose::IDENTITY)
//!     .fixed("flange", "camera", calibration.camera_pose())
//!     .build()?;
//! # let _ = tree;
//! # Ok::<_, Box<dyn std::error::Error>>(())
//! ```
//!
//! Stations should rotate the flange about at least two clearly different
//! axes, by more than a few degrees each; rotations about one axis only leave
//! the solution undetermined and are refused as [`CalibrationError::Degenerate`].

use nalgebra as na;
use roplat_msgs::Pose;

/// Where the camera is mounted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HandEye {
    /// On the flange, looking at a target fixed in the cell.
    EyeInHand,
    /// Fixed in the cell, looking at a target on the flange.
    EyeToHand,
}

/// One station: the arm and the target seen at the same instant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HandEyeSample {
    /// The flange in the robot's base frame.
    pub flange: na::Isometry3<f64>,
    /// The target in the camera's optical frame.
    pub target: na::Isometry3<f64>,
}

impl HandEyeSample {
    pub fn from_poses(flange: Pose, target: Pose) -> Self {
        Self { flange: flange.into(), target: target.into() }
    }
}

/// A solved mount; see the [module](self) docs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HandEyeCalibration {
    /// Eye in hand: the camera's optical frame in the flange frame. Eye to
    /// hand: in the robot's base frame.
    pub camera: na::Isometry3<f64>,
    /// Eye in hand: the target in the base frame. Eye to hand: in the flange
    /// frame. The mean over the samples.
    pub target: na::Isometry3<f64>,
    /// Root mean square of how far each sample puts the target from
    /// [`target`](Self::target): rotation in radians, translation in metres.
    /// Detector noise, kinematic error and unsynchronised samples all show
    /// here.
    pub rotation_rms: f64,
    pub translation_rms: f64,
    /// Station pairs that determined the rotation.
    pub pairs: usize,
}

impl HandEyeCalibration {
    /// [`camera`](Self::camera) as a message, for
    /// [`FrameTreeBuilder::fixed`](crate::FrameTreeBuilder::fixed).
    pub fn camera_pose(&self) -> Pose {
        Pose::from_isometry(self.camera)
    }
}

/// Why a calibration was refused.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum CalibrationError {
    #[error("hand-eye calibration needs at least 3 samples, got {0}")]
    TooFewSamples(usize),
    #[error("sample {0} is not finite")]
    NonFinite(usize),
    #[error(
        "the stations do not rotate the flange about two clearly different axes by more than {min_degrees} degrees, so the mount is undetermined"
    )]
    Degenerate { min_degrees: f64 },
}

/// Relative rotations smaller than this, or this close to a half turn (where
/// the rotation vector's sign is ambiguous), do not enter the rotation fit.
const MIN_ROTATION: f64 = 2.0 * std::f64::consts::PI / 180.0;
/// Smallest ratio of the rotation fit's second singular value to its first.
const MIN_SPREAD: f64 = 1e-3;

/// Solves the camera mount from `samples`; see the [module](self) docs.
pub fn calibrate_hand_eye(
    samples: &[HandEyeSample],
    setup: HandEye,
) -> Result<HandEyeCalibration, CalibrationError> {
    if samples.len() < 3 {
        return Err(CalibrationError::TooFewSamples(samples.len()));
    }
    if let Some(index) = samples
        .iter()
        .position(|s| !finite(&s.flange) || !finite(&s.target))
    {
        return Err(CalibrationError::NonFinite(index));
    }
    let degenerate = CalibrationError::Degenerate { min_degrees: MIN_ROTATION.to_degrees() };
    // Eye to hand is eye in hand with the flange poses inverted: with
    // G' = G⁻¹ both read G'_k X C_k = constant.
    let flange: Vec<na::Isometry3<f64>> = samples
        .iter()
        .map(|s| match setup {
            HandEye::EyeInHand => s.flange,
            HandEye::EyeToHand => s.flange.inverse(),
        })
        .collect();
    let target: Vec<na::Isometry3<f64>> = samples.iter().map(|s| s.target).collect();

    // A_ij X = X B_ij with A_ij = G_j⁻¹ G_i and B_ij = C_j C_i⁻¹.
    let mut motions = Vec::with_capacity(samples.len() * (samples.len() - 1) / 2);
    for i in 0..samples.len() {
        for j in i + 1..samples.len() {
            motions.push((
                flange[j].inverse() * flange[i],
                target[j] * target[i].inverse(),
            ));
        }
    }

    // Rotation: the rotation vectors satisfy α = R_X β; fit R_X to them.
    let mut h = na::Matrix3::zeros();
    let mut pairs = 0;
    for (a, b) in &motions {
        let (alpha, beta) = (a.rotation.scaled_axis(), b.rotation.scaled_axis());
        let angle = alpha.norm();
        if angle < MIN_ROTATION || angle > std::f64::consts::PI - MIN_ROTATION {
            continue;
        }
        h += beta * alpha.transpose();
        pairs += 1;
    }
    let svd = h.svd(true, true);
    let (Some(u), Some(v_t)) = (svd.u, svd.v_t) else {
        return Err(degenerate);
    };
    let mut spread = svd.singular_values.as_slice().to_vec();
    spread.sort_by(|a, b| b.total_cmp(a));
    // Written so that NaN counts as degenerate.
    let spread_out = spread[1] > MIN_SPREAD * spread[0];
    if pairs < 2 || !spread_out {
        return Err(degenerate);
    }
    let v = v_t.transpose();
    let d = (v * u.transpose()).determinant().signum();
    let rotation = v * na::Matrix3::from_diagonal(&na::Vector3::new(1.0, 1.0, d)) * u.transpose();
    let rotation = na::UnitQuaternion::from_matrix(&rotation);

    // Translation: (R_A − I) t_X = R_X t_B − t_A, by normal equations.
    let (mut m, mut r) = (na::Matrix3::zeros(), na::Vector3::zeros());
    for (a, b) in &motions {
        let k = a.rotation.to_rotation_matrix().into_inner() - na::Matrix3::identity();
        let rhs = rotation * b.translation.vector - a.translation.vector;
        m += k.transpose() * k;
        r += k.transpose() * rhs;
    }
    let translation = m.cholesky().ok_or(degenerate)?.solve(&r);
    let camera = na::Isometry3::from_parts(translation.into(), rotation);

    // Where each sample puts the target, their mean and spread.
    let placed: Vec<na::Isometry3<f64>> = flange
        .iter()
        .zip(&target)
        .map(|(g, c)| g * camera * c)
        .collect();
    let target = mean(&placed);
    let n = placed.len() as f64;
    let rotation_rms = (placed
        .iter()
        .map(|p| p.rotation.angle_to(&target.rotation).powi(2))
        .sum::<f64>()
        / n)
        .sqrt();
    let translation_rms = (placed
        .iter()
        .map(|p| (p.translation.vector - target.translation.vector).norm_squared())
        .sum::<f64>()
        / n)
        .sqrt();
    Ok(HandEyeCalibration { camera, target, rotation_rms, translation_rms, pairs })
}

fn finite(pose: &na::Isometry3<f64>) -> bool {
    pose.translation
        .vector
        .iter()
        .chain(pose.rotation.coords.iter())
        .all(|c| c.is_finite())
}

/// Mean translation, and the normalised sum of the quaternions on the first
/// one's hemisphere: close to the rotation mean for the small spread of a
/// calibration.
fn mean(poses: &[na::Isometry3<f64>]) -> na::Isometry3<f64> {
    let n = poses.len() as f64;
    let first = poses[0].rotation.coords;
    let (mut t, mut q) = (na::Vector3::zeros(), na::Vector4::zeros());
    for p in poses {
        t += p.translation.vector / n;
        let c = p.rotation.coords;
        q += if c.dot(&first) < 0.0 { -c } else { c };
    }
    na::Isometry3::from_parts(
        t.into(),
        na::UnitQuaternion::from_quaternion(na::Quaternion::from(q)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic samples in [-1, 1).
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 52) as f64 - 1.0
        }
        fn pose(&mut self, reach: f64, turn: f64) -> na::Isometry3<f64> {
            na::Isometry3::from_parts(
                na::Translation3::new(
                    reach * self.next(),
                    reach * self.next(),
                    reach * self.next(),
                ),
                na::UnitQuaternion::from_scaled_axis(
                    na::Vector3::new(self.next(), self.next(), self.next()) * turn,
                ),
            )
        }
    }

    fn mount() -> na::Isometry3<f64> {
        na::Isometry3::from_parts(
            na::Translation3::new(0.05, -0.02, 0.07),
            na::UnitQuaternion::from_euler_angles(0.1, -0.2, 1.6),
        )
    }

    fn stations(rng: &mut Lcg, n: usize) -> Vec<na::Isometry3<f64>> {
        let home = na::Isometry3::translation(0.4, 0.0, 0.5);
        (0..n).map(|_| home * rng.pose(0.15, 0.6)).collect()
    }

    fn assert_close(a: &na::Isometry3<f64>, b: &na::Isometry3<f64>, metres: f64, radians: f64) {
        let dt = (a.translation.vector - b.translation.vector).norm();
        let dr = a.rotation.angle_to(&b.rotation);
        assert!(
            dt < metres && dr < radians,
            "off by {dt} m and {dr} rad:\n{a}\n{b}"
        );
    }

    #[test]
    fn recovers_an_eye_in_hand_mount_exactly() {
        let mut rng = Lcg(7);
        let (camera, target) = (mount(), na::Isometry3::translation(0.7, 0.2, 0.0));
        let samples: Vec<_> = stations(&mut rng, 8)
            .into_iter()
            .map(|flange| HandEyeSample { flange, target: (flange * camera).inverse() * target })
            .collect();
        let c = calibrate_hand_eye(&samples, HandEye::EyeInHand).unwrap();
        assert_close(&c.camera, &camera, 1e-9, 1e-9);
        assert_close(&c.target, &target, 1e-9, 1e-9);
        assert!(c.translation_rms < 1e-9 && c.rotation_rms < 1e-9, "{c:?}");
        assert_eq!(c.pairs, 28);
    }

    #[test]
    fn recovers_an_eye_to_hand_mount_exactly() {
        let mut rng = Lcg(11);
        // The camera above the cell; the target held 10 cm out from the flange.
        let camera = na::Isometry3::from_parts(
            na::Translation3::new(1.2, 0.0, 0.9),
            na::UnitQuaternion::from_euler_angles(std::f64::consts::PI, 0.4, 0.0),
        );
        let held = na::Isometry3::translation(0.0, 0.0, 0.1);
        let samples: Vec<_> = stations(&mut rng, 8)
            .into_iter()
            .map(|flange| HandEyeSample { flange, target: camera.inverse() * flange * held })
            .collect();
        let c = calibrate_hand_eye(&samples, HandEye::EyeToHand).unwrap();
        assert_close(&c.camera, &camera, 1e-9, 1e-9);
        assert_close(&c.target, &held, 1e-9, 1e-9);
    }

    #[test]
    fn noisy_detections_land_near_the_mount_and_show_in_the_spread() {
        let mut rng = Lcg(23);
        let (camera, target) = (mount(), na::Isometry3::translation(0.7, 0.2, 0.0));
        // 1 mm and about 0.2 degrees of detector noise, 20 stations.
        let samples: Vec<_> = stations(&mut rng, 20)
            .into_iter()
            .map(|flange| {
                let noise = rng.pose(0.001, 0.003);
                HandEyeSample { flange, target: noise * (flange * camera).inverse() * target }
            })
            .collect();
        let c = calibrate_hand_eye(&samples, HandEye::EyeInHand).unwrap();
        assert_close(&c.camera, &camera, 0.005, 0.01);
        assert!(
            c.translation_rms > 1e-4 && c.translation_rms < 0.01,
            "{c:?}"
        );
        assert!(c.rotation_rms > 1e-4 && c.rotation_rms < 0.02, "{c:?}");
    }

    #[test]
    fn refuses_too_few_non_finite_and_single_axis_stations() {
        let mut rng = Lcg(3);
        let (camera, target) = (mount(), na::Isometry3::translation(0.7, 0.2, 0.0));
        let sample = |flange: na::Isometry3<f64>| HandEyeSample {
            flange,
            target: (flange * camera).inverse() * target,
        };
        let two: Vec<_> = stations(&mut rng, 2).into_iter().map(sample).collect();
        assert_eq!(
            calibrate_hand_eye(&two, HandEye::EyeInHand),
            Err(CalibrationError::TooFewSamples(2))
        );

        let mut bad: Vec<_> = stations(&mut rng, 4).into_iter().map(sample).collect();
        bad[2].target.translation.x = f64::NAN;
        assert_eq!(
            calibrate_hand_eye(&bad, HandEye::EyeInHand),
            Err(CalibrationError::NonFinite(2))
        );

        // Turning only about the flange's z axis cannot fix the mount.
        let spun: Vec<_> = (0..6)
            .map(|k| {
                na::Isometry3::from_parts(
                    na::Translation3::new(0.4 + 0.02 * k as f64, 0.0, 0.5),
                    na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3 * k as f64),
                )
            })
            .map(sample)
            .collect();
        assert!(matches!(
            calibrate_hand_eye(&spun, HandEye::EyeInHand),
            Err(CalibrationError::Degenerate { .. })
        ));
    }
}

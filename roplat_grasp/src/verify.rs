//! Whether a grasp holds, judged the same way for every parallel gripper.
//!
//! Drivers report a grasp differently: Franka's hand compares the stopped
//! width with the commanded one, a Robotiq gripper detects an object from its
//! motor current, a simulated hand compares widths. What they all report is
//! the finger opening, so [`GraspCheck`] judges that against the width the
//! grasp expects and takes a driver's own flag (`GripperState::grasped`) as a
//! second opinion. After the lift, [`carried_mass`] tells from a wrist force
//! sensor whether the object came along.
//!
//! ```
//! use roplat_grasp::{GraspCheck, GraspOutcome, carried_mass};
//!
//! // A 40 mm block, judged within 3 mm.
//! let check = GraspCheck::new(0.04, 0.003);
//! // After closing: the hand reports 39.2 mm and that it grasps.
//! assert!(check.judge(0.0392, Some(true)).is_held());
//! // Closed on nothing.
//! assert!(matches!(check.judge(0.0004, Some(false)), GraspOutcome::Empty { .. }));
//!
//! // Lifted: the wrist (here level, gravity along its -z) pulls 2.45 N more.
//! let mass = carried_mass([0.1, 0.0, -5.0], [0.1, 0.0, -7.45], [0.0, 0.0, -9.81]);
//! assert!((mass - 0.25).abs() < 0.01);
//! ```

/// What a [`GraspCheck`] concluded from the finger opening.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GraspOutcome {
    /// The fingers stopped on something of the expected width.
    Held { width: f64 },
    /// The fingers closed (almost) fully: nothing is between them, or the
    /// object slipped out.
    Empty { width: f64 },
    /// The fingers stopped at another width: another object, another face
    /// of it, or the object turned in the hand.
    Unexpected { width: f64, expected: f64 },
    /// The opening matches but the driver reports no grasp. Its own check
    /// (current, force) saw something the width does not show, such as a
    /// grip too weak to hold.
    Disputed { width: f64 },
}

impl GraspOutcome {
    pub fn is_held(&self) -> bool {
        matches!(self, Self::Held { .. })
    }
}

/// The opening a grasp should end at; see the [module](self) docs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraspCheck {
    /// Expected opening on the object (m).
    pub width: f64,
    /// How far the measured opening may be from `width` (m): the object's
    /// size tolerance, finger compliance and the gripper's resolution.
    pub tolerance: f64,
    /// An opening at or below this counts as closed on nothing (m).
    pub empty_below: f64,
}

impl GraspCheck {
    /// Expecting `width` within `tolerance`. Closed on nothing below 2 mm,
    /// or below half of `width` for a thinner object.
    pub fn new(width: f64, tolerance: f64) -> Self {
        Self { width, tolerance, empty_below: (0.5 * width).min(0.002) }
    }

    /// Expecting the width a [`Grasp`](crate::Grasp) candidate was made for.
    pub fn for_grasp(grasp: &crate::Grasp, tolerance: f64) -> Self {
        Self::new(grasp.width, tolerance)
    }

    /// Judges the opening `width` (m) after closing, with the driver's own
    /// verdict if it gives one. A NaN width is unexpected.
    pub fn judge(&self, width: f64, reported: Option<bool>) -> GraspOutcome {
        if width <= self.empty_below {
            return GraspOutcome::Empty { width };
        }
        if (width - self.width).abs() <= self.tolerance {
            return match reported {
                Some(false) => GraspOutcome::Disputed { width },
                _ => GraspOutcome::Held { width },
            };
        }
        GraspOutcome::Unexpected { width, expected: self.width }
    }
}

/// The mass (kg) the wrist took on between two force readings `before` and
/// `after` (N, in the sensor frame), with the wrist in the same orientation
/// for both, as just before and just after a vertical lift. `gravity` is
/// gravity's acceleration in the sensor frame (m/s²), from the wrist's
/// orientation: `R_world_sensor⁻¹ · (0, 0, −9.81)` for a z-up world.
///
/// The sensor is taken to report the force the tool exerts on it, so a held
/// load adds force along gravity; for a sensor that reports the reaction,
/// pass the readings the other way round. Forces across gravity, such as a
/// finger still touching a neighbour, do not count. Zero for zero gravity.
pub fn carried_mass(before: [f64; 3], after: [f64; 3], gravity: [f64; 3]) -> f64 {
    let g2: f64 = gravity.iter().map(|g| g * g).sum();
    if g2 == 0.0 {
        return 0.0;
    }
    let along: f64 = (0..3).map(|i| (after[i] - before[i]) * gravity[i]).sum();
    along / g2
}

#[cfg(test)]
mod tests {
    use nalgebra as na;

    use super::*;

    #[test]
    fn judges_the_opening_and_weighs_the_driver_flag() {
        let check = GraspCheck::new(0.04, 0.003);
        assert_eq!(check.empty_below, 0.002);
        assert_eq!(
            check.judge(0.0395, None),
            GraspOutcome::Held { width: 0.0395 }
        );
        assert_eq!(
            check.judge(0.0428, Some(true)),
            GraspOutcome::Held { width: 0.0428 }
        );
        assert_eq!(
            check.judge(0.0395, Some(false)),
            GraspOutcome::Disputed { width: 0.0395 }
        );
        assert_eq!(
            check.judge(0.001, Some(true)),
            GraspOutcome::Empty { width: 0.001 }
        );
        assert_eq!(check.judge(0.0, None), GraspOutcome::Empty { width: 0.0 });
        assert_eq!(
            check.judge(0.025, Some(true)),
            GraspOutcome::Unexpected { width: 0.025, expected: 0.04 }
        );
        assert!(matches!(
            check.judge(f64::NAN, None),
            GraspOutcome::Unexpected { .. }
        ));
        // A thin object: closed on nothing only below half its width.
        let thin = GraspCheck::new(0.003, 0.001);
        assert_eq!(thin.empty_below, 0.0015);
        assert!(thin.judge(0.0028, None).is_held());
    }

    #[test]
    fn a_candidate_carries_its_width() {
        let grasp = crate::Grasp { pose: na::Isometry3::identity(), width: 0.06, score: 1.0 };
        let check = GraspCheck::for_grasp(&grasp, 0.002);
        assert_eq!((check.width, check.tolerance), (0.06, 0.002));
        assert!(check.judge(0.0605, None).is_held());
    }

    #[test]
    fn the_carried_mass_is_the_force_change_along_gravity() {
        // The wrist pointing down and turned: gravity in the sensor frame
        // from its orientation.
        let wrist = na::UnitQuaternion::from_euler_angles(std::f64::consts::PI, 0.3, 1.2);
        let g = wrist.inverse() * na::Vector3::new(0.0, 0.0, -9.81);
        let gravity = [g.x, g.y, g.z];
        let before = [1.0, -2.0, 3.0];
        // 0.5 kg more along gravity, plus a sideways push that must not count.
        let side = g.cross(&na::Vector3::x()).normalize() * 4.0;
        let after: [f64; 3] = std::array::from_fn(|i| before[i] + 0.5 * g[i] + side[i]);
        assert!((carried_mass(before, after, gravity) - 0.5).abs() < 1e-12);
        // Dropped: back to the bare reading.
        assert!(carried_mass(before, before, gravity).abs() < 1e-12);
        assert_eq!(carried_mass(before, after, [0.0; 3]), 0.0);
    }
}

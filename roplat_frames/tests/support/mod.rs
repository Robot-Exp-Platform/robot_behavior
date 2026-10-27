//! A planar two-joint arm on a calibrated base, with a hand camera watching a
//! mug. Each test binary uses part of it.
#![allow(dead_code)]

use std::f64::consts::FRAC_1_SQRT_2;

use roplat_description::RobotModel;
use roplat_frames::{FrameId, FrameTree, History, frame_id};
use roplat_msgs::{Pose, Quaternion, Stamp, Vector3, clock};

/// Shoulder and elbow about z, a fixed wrist that turns the hand a quarter
/// about y, and a prismatic finger that no group positions.
pub const ARM: &str = r#"
<robot name="cell_arm">
  <link name="base"/>
  <link name="upper"/>
  <link name="fore"/>
  <link name="hand"/>
  <link name="finger"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="upper"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" velocity="2" effort="40"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper"/><child link="fore"/><origin xyz="0.4 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" velocity="2" effort="20"/>
  </joint>
  <joint name="wrist_mount" type="fixed">
    <parent link="fore"/><child link="hand"/><origin xyz="0.3 0 0" rpy="0 1.5707963267948966 0"/>
  </joint>
  <joint name="finger" type="prismatic">
    <parent link="hand"/><child link="finger"/><axis xyz="0 1 0"/>
    <limit lower="0" upper="0.04" velocity="0.1" effort="10"/>
  </joint>
</robot>
"#;

pub const WORLD: FrameId = frame_id("world");
pub const BASE: FrameId = frame_id("base");
pub const FORE: FrameId = frame_id("fore");
pub const HAND: FrameId = frame_id("hand");
pub const FINGER: FrameId = frame_id("finger");
pub const CAMERA: FrameId = frame_id("camera");
pub const MUG: FrameId = frame_id("mug");

/// Calibrations are compile-time values too.
pub const BASE_IN_WORLD: Pose = Pose {
    position: Vector3::new(1., 0., 0.),
    orientation: Quaternion { x: 0., y: 0., z: FRAC_1_SQRT_2, w: FRAC_1_SQRT_2 },
};
pub const CAMERA_IN_HAND: Pose = Pose {
    position: Vector3::new(0.05, 0., 0.02),
    orientation: Quaternion::IDENTITY,
};

pub fn model() -> RobotModel {
    RobotModel::from_urdf_str(ARM).unwrap()
}

/// Base-to-hand pose from the description's own chain.
pub fn hand_in_base(q: [f64; 2]) -> Pose {
    model()
        .chain("base", "hand")
        .unwrap()
        .forward_kinematics(&q)
        .unwrap()
}

pub fn cell(arm: History, mug: History) -> FrameTree {
    FrameTree::builder()
        .fixed("world", "base", BASE_IN_WORLD)
        .model(&model())
        .fixed("hand", "camera", CAMERA_IN_HAND)
        .joints("arm", &["shoulder", "elbow"], arm)
        .dynamic("camera", "mug", mug)
        .build()
        .unwrap()
}

/// `ms` milliseconds on the monotonic clock.
pub fn ms(ms: f64) -> Stamp {
    Stamp::new(clock::MONOTONIC, (ms * 1e6).round() as i64)
}

#[track_caller]
pub fn assert_pose(actual: Pose, expected: Pose) {
    let (a, e) = (actual.quat(), expected.quat());
    let offset = (a.translation.vector - e.translation.vector).norm();
    let turn = a.rotation.angle_to(&e.rotation);
    assert!(offset < 1e-9 && turn < 1e-9, "{actual:?} != {expected:?}");
}

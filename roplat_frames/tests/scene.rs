//! Scene objects and depth readings placed where the buffer had their frames.
#![cfg(feature = "scene")]

mod support;

use std::path::PathBuf;

use nalgebra::Isometry3;
use roplat_frames::scene::{SceneSyncError, add_sensed_points, place_objects};
use roplat_frames::{FrameBuffer, FrameError, History};
use roplat_msgs::{Pose, Transform, clock};
use roplat_scene::PlanningScene;
use support::*;

/// The cell's buffer: the arm at rest at 0 ms and turned at 10 ms, the mug
/// seen at the same place in the hand camera at both.
fn recorded() -> FrameBuffer {
    let mut frames =
        FrameBuffer::new(cell(History::new(8), History::new(8)), clock::MONOTONIC).unwrap();
    let group = frames.tree().group::<2>("arm").unwrap();
    let mug = Transform { parent: CAMERA, child: MUG, pose: MUG_IN_CAMERA };
    for (t, q) in [(0.0, [0.0, 0.0]), (10.0, [0.4, -0.3])] {
        frames.set_joints(group, ms(t), &q).unwrap();
        frames.set_transform(ms(t), &mug).unwrap();
    }
    frames
}

const MUG_IN_CAMERA: Pose = Pose {
    position: roplat_msgs::Vector3::new(0.0, 0.0, 0.3),
    orientation: roplat_msgs::Quaternion::IDENTITY,
};

fn scene() -> PlanningScene<2> {
    let mut scene = PlanningScene::new(model(), ["shoulder", "elbow"], |name: &str| {
        PathBuf::from(name)
    })
    .unwrap();
    scene.set_base(BASE_IN_WORLD.into());
    scene
}

/// Where the hand camera is in the world with the arm at `q`.
fn camera_in_world(q: [f64; 2]) -> Isometry3<f64> {
    Isometry3::from(BASE_IN_WORLD)
        * Isometry3::from(hand_in_base(q))
        * Isometry3::from(CAMERA_IN_HAND)
}

fn assert_close(a: &Isometry3<f64>, b: &Isometry3<f64>) {
    let dt = (a.translation.vector - b.translation.vector).norm();
    let dr = a.rotation.angle_to(&b.rotation);
    assert!(
        dt < 1e-9 && dr < 1e-9,
        "off by {dt} m and {dr} rad:\n{a}\n{b}"
    );
}

#[test]
fn a_tracked_object_moves_to_its_frame_at_the_stamp() {
    let frames = recorded();
    let mut scene = scene();
    let cup = roplat_description::Geometry::Sphere { radius: 0.04 };
    scene
        .add_object("mug", &cup, Isometry3::identity())
        .unwrap();
    // Halfway between the two joint samples the arm is at the midpoint.
    place_objects(&frames, &mut scene, WORLD, ms(5.0), &[("mug", MUG)]).unwrap();
    let expected = camera_in_world([0.2, -0.15]) * Isometry3::from(MUG_IN_CAMERA);
    assert_close(&scene.object_pose("mug", &[0.0; 2]).unwrap(), &expected);

    // Past the last sample the buffer refuses, and the object stays.
    let late = place_objects(&frames, &mut scene, WORLD, ms(20.0), &[("mug", MUG)]);
    assert!(
        matches!(
            late,
            Err(SceneSyncError::Frame { source: FrameError::AfterLatest { .. }, .. })
        ),
        "{late:?}"
    );
    assert_close(&scene.object_pose("mug", &[0.0; 2]).unwrap(), &expected);
    // An object the scene does not have.
    let missing = place_objects(&frames, &mut scene, WORLD, ms(5.0), &[("cup", MUG)]);
    assert!(
        matches!(missing, Err(SceneSyncError::Scene { .. })),
        "{missing:?}"
    );
}

#[test]
fn a_depth_reading_lands_at_the_camera_pose_of_its_stamp() {
    let frames = recorded();
    let mut scene = scene();
    // A 5 x 5 patch 0.4 m in front of the camera.
    let points: Vec<[f64; 3]> = (0..25)
        .map(|k| [0.02 * (k % 5) as f64, 0.02 * (k / 5) as f64, 0.4])
        .collect();
    let voxels = add_sensed_points(
        &frames,
        &mut scene,
        "seen",
        &points,
        0.02,
        WORLD,
        CAMERA,
        ms(10.0),
    )
    .unwrap();
    assert!(voxels >= 9, "{voxels} voxels");
    // The reading is placed where the camera was at 10 ms, not at rest.
    let placed = scene.object_pose("seen", &[0.0; 2]).unwrap();
    assert_close(&placed, &camera_in_world([0.4, -0.3]));
    assert!(
        (placed.translation.vector - camera_in_world([0.0, 0.0]).translation.vector).norm() > 0.05
    );

    // A stamp on another clock is refused before anything is added.
    let simulated = roplat_msgs::Stamp::new(clock::SIMULATION, 10_000_000);
    let wrong = add_sensed_points(
        &frames, &mut scene, "again", &points, 0.02, WORLD, CAMERA, simulated,
    );
    assert!(
        matches!(
            wrong,
            Err(SceneSyncError::Frame { source: FrameError::ClockMismatch { .. }, .. })
        ),
        "{wrong:?}"
    );
    assert!(!scene.object_names().any(|name| name == "again"));
}

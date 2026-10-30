//! A wall seen by a depth camera becomes an obstacle the planner avoids.

use std::path::PathBuf;

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};
use roplat_description::RobotModel;
use roplat_msgs::CameraInfo;
use roplat_scene::{CollisionCheck, PlanningScene, SceneError, points::depth_to_points};
use roplat_trajectory::{PlannerOptions, plan_joint_path, segment_is_valid};

/// A planar two-link arm 0.25 m up: 0.5 m upper arm, 0.4 m forearm and a
/// finger, 0.93 m from the shoulder when straight (as in `scene.rs`).
const ARM: &str = r#"<robot name="toy">
  <link name="base"><collision><origin xyz="0 0 0.1"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/><origin xyz="0 0 0.25"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="upper"><collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.05 0.05"/></geometry></collision></link>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/><origin xyz="0.5 0 0"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="fore"><collision><origin xyz="0.2 0 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder radius="0.03" length="0.4"/></geometry></collision></link>
  <joint name="wrist" type="fixed"><parent link="fore"/><child link="finger"/><origin xyz="0.43 0 0"/></joint>
  <link name="finger"><collision><geometry><sphere radius="0.02"/></geometry></collision></link>
</robot>"#;

fn scene() -> PlanningScene<2> {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    PlanningScene::new(model, ["shoulder", "elbow"], |name: &str| {
        PathBuf::from(name)
    })
    .unwrap()
}

/// A camera at (1.5, 0, 0.25) looking back along -x: optical z is world -x,
/// optical y (down) is world -z, optical x is world +y.
fn camera_pose() -> Isometry3<f64> {
    let columns = Matrix3::from_columns(&[Vector3::y(), -Vector3::z(), -Vector3::x()]);
    let rotation = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(columns));
    Isometry3::from_parts(Translation3::new(1.5, 0.0, 0.25), rotation)
}

/// 21 x 21 pixels, focal length 20 px, every pixel 0.9 m away: a wall at
/// world x = 0.6 spanning 0.45 m each way around the camera's axis.
fn wall_image() -> (Vec<f32>, CameraInfo) {
    let info = CameraInfo {
        width: 21,
        height: 21,
        k: [20.0, 0.0, 10.0, 0.0, 20.0, 10.0, 0.0, 0.0, 1.0],
        ..Default::default()
    };
    (vec![0.9; 21 * 21], info)
}

#[test]
fn a_sensed_wall_blocks_the_straight_arm() {
    let mut scene = scene();
    let (depth, info) = wall_image();
    let points = depth_to_points(&depth, &info, 0.1..=3.0, 1);
    assert_eq!(points.len(), 21 * 21);
    let voxels = scene
        .add_points("wall", &points, 0.03, camera_pose())
        .unwrap();
    assert!(voxels > 100, "{voxels} voxels");

    // Straight out along +x the forearm passes x = 0.6: blocked by the wall.
    let contacts = scene.contacts(&[0.0, 0.0]);
    assert!(
        contacts
            .iter()
            .any(|c| c.first == "wall" || c.second == "wall"),
        "{contacts:?}"
    );
    // Pointing along +y, or folded back, it is clear.
    assert!(scene.is_free(&[std::f64::consts::FRAC_PI_2, 0.0]));
    assert!(scene.is_free(&[0.0, 2.5]));

    // The names stay unique, and an empty reading is refused.
    assert!(matches!(
        scene.add_points("wall", &points, 0.03, camera_pose()),
        Err(SceneError::NameTaken(_))
    ));
    assert!(matches!(
        scene.add_points("empty", &[], 0.03, camera_pose()),
        Err(SceneError::InvalidGeometry(_))
    ));
    assert!(matches!(
        scene.add_points("bad", &points, 0.0, camera_pose()),
        Err(SceneError::InvalidGeometry(_))
    ));

    // Replacing the reading: remove and add again.
    scene.remove_object("wall").unwrap();
    assert!(scene.is_free(&[0.0, 0.0]));
}

#[test]
fn the_planner_folds_the_arm_past_the_sensed_wall() {
    let mut scene = scene();
    let (depth, info) = wall_image();
    scene
        .add_points(
            "wall",
            &depth_to_points(&depth, &info, 0.1..=3.0, 1),
            0.03,
            camera_pose(),
        )
        .unwrap();
    let (start, goal) = ([-1.4, 0.0], [1.4, 0.0]);
    let options = PlannerOptions { seed: 3, ..PlannerOptions::default() };
    // Swinging straight across sweeps through the wall.
    assert!(!segment_is_valid(&start, &goal, options.resolution, |q| {
        scene.is_free(q)
    }));
    let path = plan_joint_path(
        start,
        goal,
        [-3.1; 2],
        [3.1; 2],
        |q| scene.is_free(q),
        options,
    )
    .unwrap();
    for pair in path.windows(2) {
        assert!(segment_is_valid(
            &pair[0],
            &pair[1],
            options.resolution,
            |q| scene.is_free(q)
        ));
    }
    // Somewhere on the way the elbow bends to shorten the reach.
    assert!(path.iter().any(|q| q[1].abs() > 0.5), "{path:?}");
}

//! An occupancy map keeps the scene current: a wall that is seen blocks
//! the arm, and once it is gone the rays through it clear it again.

use std::path::PathBuf;

use nalgebra::Isometry3;
use roplat_description::{Geometry, RobotModel};
use roplat_scene::{CollisionCheck, PlanningScene, SceneError, occupancy::OccupancyMap};

/// A planar two-link arm 0.25 m up, reaching 0.95 m when straight (as in
/// `scene.rs`).
const ARM: &str = r#"<robot name="toy">
  <link name="base"><collision><origin xyz="0 0 0.1"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/><origin xyz="0 0 0.25"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="upper"><collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.05 0.05"/></geometry></collision></link>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/><origin xyz="0.5 0 0"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="fore"><collision><origin xyz="0.2 0 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder radius="0.03" length="0.4"/></geometry></collision></link>
  <joint name="wrist" type="fixed"><parent link="fore"/><child link="finger"/><origin xyz="0.43 0 0"/></joint>
  <link name="finger"><collision><geometry><sphere radius="0.02"/></geometry></collision></link>
</robot>"#;

const STRAIGHT: [f64; 2] = [0.0, 0.0];
/// The scanner, beyond the arm's reach, looking back along -x.
const SCANNER: [f64; 3] = [1.6, 0.0, 0.25];

fn scene() -> PlanningScene<2> {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    PlanningScene::new(model, ["shoulder", "elbow"], |name: &str| {
        PathBuf::from(name)
    })
    .unwrap()
}

/// One scan: rays from the scanner through a grid on the plane x = 0.7,
/// ±0.3 m across and ±0.15 m up and down around the arm's height. Each ray
/// returns from the wall on that plane if `wall`, else from a far plane at
/// x = -1.2, behind the robot.
fn scan(wall: bool) -> Vec<[f64; 3]> {
    let far = (SCANNER[0] + 1.2) / (SCANNER[0] - 0.7);
    let mut points = Vec::new();
    for i in 0..=30 {
        for j in 0..=15 {
            let on_wall = [0.7, -0.3 + 0.02 * i as f64, 0.1 + 0.02 * j as f64];
            let t = if wall { 1.0 } else { far };
            points.push(std::array::from_fn(|k| {
                SCANNER[k] + (on_wall[k] - SCANNER[k]) * t
            }));
        }
    }
    points
}

#[test]
fn a_wall_seen_blocks_the_arm_and_clears_when_gone() {
    let mut scene = scene();
    let mut map = OccupancyMap::new(0.04);
    for _ in 0..3 {
        map.insert_scan(SCANNER, &scan(true), 5.0);
    }
    let voxels = scene.set_occupancy("seen", &map).unwrap();
    assert!(voxels > 100, "{voxels} voxels");
    // The straight arm reaches 0.95 m, through the wall at 0.7 m.
    let contact = scene.first_contact(&STRAIGHT).unwrap();
    assert_eq!(contact.second, "seen");
    // Folded back it clears the wall.
    assert!(scene.is_free(&[3.0, 0.0]));

    // The wall is taken away: the rays now pass through it to the far plane.
    let mut scans = 0;
    loop {
        map.insert_scan(SCANNER, &scan(false), 5.0);
        scene.set_occupancy("seen", &map).unwrap();
        scans += 1;
        if scene.is_free(&STRAIGHT) {
            break;
        }
        assert!(scans < 8, "the wall did not clear");
    }
    assert!(
        scans >= 2,
        "a single scan should not overturn three: {scans}"
    );
    // The far plane is in the map now (a ray returns at y = 0, z = 0.281);
    // the space behind it was never seen.
    assert_eq!(map.is_occupied([-1.19, 0.0, 0.281]), Some(true));
    assert_eq!(map.is_occupied([-1.5, 0.0, 0.25]), None);
    // One object, replaced in place each time.
    assert_eq!(scene.object_names().collect::<Vec<_>>(), ["seen"]);
}

#[test]
fn occupancy_takes_only_world_names() {
    let mut scene = scene();
    let map = OccupancyMap::new(0.05);
    assert!(matches!(
        scene.set_occupancy("upper", &map),
        Err(SceneError::NameTaken(_))
    ));
    scene
        .add_object(
            "cup",
            &Geometry::Sphere { radius: 0.03 },
            Isometry3::translation(0.95, 0.0, 0.25),
        )
        .unwrap();
    scene.attach("cup", "finger", &[], &STRAIGHT).unwrap();
    assert!(matches!(
        scene.set_occupancy("cup", &map),
        Err(SceneError::Attached(_))
    ));
    // An empty map leaves no object behind.
    assert_eq!(scene.set_occupancy("seen", &map).unwrap(), 0);
    assert_eq!(scene.object_names().collect::<Vec<_>>(), ["cup"]);
}

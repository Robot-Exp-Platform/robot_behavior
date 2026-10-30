//! The robot's own geometry: a concave link keeps its hollow once
//! decomposed, and sensed points on the robot are filtered out.

use std::{
    f64::consts::FRAC_PI_2,
    fmt::Write as _,
    path::{Path, PathBuf},
    sync::OnceLock,
};

use nalgebra::{Isometry3, Point3, Vector3};
use roplat_description::{Geometry, RobotModel};
use roplat_scene::{CollisionCheck, PlanningScene, SceneError};

/// A box open towards -x with 2 cm walls: outside x 0.73–1.07, |y| ≤ 0.12,
/// z 0.13–0.37; hollow to x = 1.05, |y| ≤ 0.10, z 0.15–0.35. An OBJ file of
/// 14 quads (as in `concave.rs`), written once: tests run in parallel and
/// must not read it while another rewrites it.
fn pigeonhole_obj() -> &'static Path {
    static PATH: OnceLock<PathBuf> = OnceLock::new();
    PATH.get_or_init(write_pigeonhole_obj)
}

fn write_pigeonhole_obj() -> PathBuf {
    let mut vertices = Vec::new();
    for (xs, half_width, zs) in [
        ([0.73, 1.07], 0.12, [0.13, 0.37]),
        ([0.73, 1.05], 0.10, [0.15, 0.35]),
    ] {
        for x in xs {
            for y in [-half_width, half_width] {
                for z in zs {
                    vertices.push([x, y, z]);
                }
            }
        }
    }
    let o = |a: u32, b: u32, c: u32| 4 * a + 2 * b + c;
    let i = |a: u32, b: u32, c: u32| 8 + o(a, b, c);
    let quads = [
        [o(1, 0, 0), o(1, 1, 0), o(1, 1, 1), o(1, 0, 1)],
        [o(0, 0, 0), o(1, 0, 0), o(1, 0, 1), o(0, 0, 1)],
        [o(0, 1, 0), o(1, 1, 0), o(1, 1, 1), o(0, 1, 1)],
        [o(0, 0, 0), o(1, 0, 0), o(1, 1, 0), o(0, 1, 0)],
        [o(0, 0, 1), o(1, 0, 1), o(1, 1, 1), o(0, 1, 1)],
        [o(0, 0, 0), o(0, 1, 0), i(0, 1, 0), i(0, 0, 0)],
        [o(0, 0, 1), o(0, 1, 1), i(0, 1, 1), i(0, 0, 1)],
        [o(0, 0, 0), o(0, 0, 1), i(0, 0, 1), i(0, 0, 0)],
        [o(0, 1, 0), o(0, 1, 1), i(0, 1, 1), i(0, 1, 0)],
        [i(1, 0, 0), i(1, 1, 0), i(1, 1, 1), i(1, 0, 1)],
        [i(0, 0, 0), i(1, 0, 0), i(1, 0, 1), i(0, 0, 1)],
        [i(0, 1, 0), i(1, 1, 0), i(1, 1, 1), i(0, 1, 1)],
        [i(0, 0, 0), i(1, 0, 0), i(1, 1, 0), i(0, 1, 0)],
        [i(0, 0, 1), i(1, 0, 1), i(1, 1, 1), i(0, 1, 1)],
    ];
    let mut text = String::from("# pigeonhole\n");
    for [x, y, z] in &vertices {
        writeln!(text, "v {x} {y} {z}").unwrap();
    }
    for [a, b, c, d] in quads {
        writeln!(text, "f {} {} {} {}", a + 1, b + 1, c + 1, d + 1).unwrap();
    }
    let path = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("roplat_scene_cup_link.obj");
    std::fs::write(&path, text).unwrap();
    path
}

/// A turntable carrying the pigeonhole as its link, with a peg on the
/// side: the hollow is the link's own.
fn cup_robot(mesh: &str) -> RobotModel {
    RobotModel::from_urdf_str(&format!(
        r#"<robot name="cup">
  <link name="base"/>
  <joint name="turn" type="revolute"><parent link="base"/><child link="cup"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="cup">
    <collision><geometry><mesh filename="{mesh}"/></geometry></collision>
    <collision><origin xyz="0.9 0.2 0.25"/><geometry><box size="0.04 0.04 0.04"/></geometry></collision>
  </link>
</robot>"#
    ))
    .unwrap()
}

fn as_path(name: &str) -> PathBuf {
    PathBuf::from(name)
}

#[test]
fn a_decomposed_link_keeps_its_hollow() {
    let mesh = pigeonhole_obj();
    let mut scene: PlanningScene<1> =
        PlanningScene::new(cup_robot(mesh.to_str().unwrap()), ["turn"], as_path).unwrap();
    // A post inside the hollow when the turntable is at zero.
    scene
        .add_object(
            "post",
            &Geometry::Box { size: [0.05, 0.05, 0.1] },
            Isometry3::translation(0.9, 0.0, 0.25),
        )
        .unwrap();
    assert!(!scene.is_free(&[0.0]), "the hull fills the hollow");
    let parts = scene.decompose_link("cup", as_path).unwrap();
    // The walls, plus the peg kept as a box.
    assert!(parts >= 6, "{parts} parts");
    assert!(scene.is_free(&[0.0]), "{:?}", scene.first_contact(&[0.0]));
    // Turned, a side wall meets the post; the motion there is caught too.
    assert!(!scene.is_free(&[0.15]));
    let hit = scene.motion_contact(&[0.0], &[0.15]).unwrap();
    assert_eq!(hit.contact.second, "post");
    assert!(hit.fraction > 0.0 && hit.fraction < 1.0);
}

#[test]
fn decomposing_refuses_what_it_cannot_read() {
    let mut scene: PlanningScene<1> = PlanningScene::new(
        cup_robot(pigeonhole_obj().to_str().unwrap()),
        ["turn"],
        as_path,
    )
    .unwrap();
    assert!(matches!(
        scene.decompose_link("nowhere", as_path),
        Err(SceneError::UnknownLink(_))
    ));
    assert!(matches!(
        scene.decompose_link("cup", |_| PathBuf::from("no/such/mesh.obj")),
        Err(SceneError::Mesh { .. })
    ));
    // A failed decomposition leaves the link as it was: the hull.
    scene
        .add_object(
            "post",
            &Geometry::Box { size: [0.05, 0.05, 0.1] },
            Isometry3::translation(0.9, 0.0, 0.25),
        )
        .unwrap();
    assert!(!scene.is_free(&[0.0]));
}

/// A planar two-link arm 0.25 m up (as in `scene.rs`).
const ARM: &str = r#"<robot name="toy">
  <link name="base"><collision><origin xyz="0 0 0.1"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/><origin xyz="0 0 0.25"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="upper"><collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.05 0.05"/></geometry></collision></link>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/><origin xyz="0.5 0 0"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="fore"><collision><origin xyz="0.2 0 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder radius="0.03" length="0.4"/></geometry></collision></link>
  <joint name="wrist" type="fixed"><parent link="fore"/><child link="finger"/><origin xyz="0.43 0 0"/></joint>
  <link name="finger"><collision><geometry><sphere radius="0.02"/></geometry></collision></link>
</robot>"#;

fn arm() -> PlanningScene<2> {
    PlanningScene::new(
        RobotModel::from_urdf_str(ARM).unwrap(),
        ["shoulder", "elbow"],
        as_path,
    )
    .unwrap()
}

/// What a camera above sees with the arm bent at `q`: the top of the upper
/// arm and of the forearm, 1 cm apart, and a table at 0.1 m around them.
fn reading(q: [f64; 2]) -> (Vec<[f64; 3]>, usize) {
    let mut on_arm = Vec::new();
    let shoulder = Isometry3::new(Vector3::new(0.0, 0.0, 0.25), Vector3::new(0.0, 0.0, q[0]));
    let elbow =
        shoulder * Isometry3::new(Vector3::new(0.5, 0.0, 0.0), Vector3::new(0.0, 0.0, q[1]));
    for k in 0..45 {
        on_arm.push(shoulder * Point3::new(0.03 + 0.01 * k as f64, 0.0, 0.025));
    }
    for k in 0..38 {
        on_arm.push(elbow * Point3::new(0.01 + 0.01 * k as f64, 0.0, 0.03));
    }
    let count = on_arm.len();
    let mut points: Vec<[f64; 3]> = on_arm.into_iter().map(|p| p.coords.into()).collect();
    for i in 0..30 {
        for j in 0..30 {
            points.push([-0.5 + 0.05 * i as f64, -0.75 + 0.05 * j as f64, 0.1]);
        }
    }
    (points, count)
}

#[test]
fn points_on_the_arm_are_filtered_and_the_rest_kept() {
    let scene = arm();
    let q = [0.4, -0.9];
    let (points, on_arm) = reading(q);
    let on = scene.points_on_robot(&points, &q, 0.01);
    assert!(
        on[..on_arm].iter().all(|&on| on),
        "every arm point is on the robot"
    );
    // The table is 15 cm below the arm, under the base block's top (0.2 m)
    // only where the block stands: those points are inside it.
    let under_base = points[on_arm..]
        .iter()
        .filter(|p| p[0].abs() <= 0.11 && p[1].abs() <= 0.11)
        .count();
    let table_on = on[on_arm..].iter().filter(|&&on| on).count();
    assert_eq!(table_on, under_base);
    let kept = scene.without_robot_points(&points, &q, 0.01);
    assert_eq!(kept.len(), points.len() - on_arm - under_base);

    // With the arm elsewhere, the same arm points are obstacles.
    let elsewhere = [-1.5, 0.9];
    let on = scene.points_on_robot(&points, &elsewhere, 0.01);
    assert!(on[..on_arm].iter().all(|&on| !on));
}

#[test]
fn the_margin_and_attached_objects_count() {
    let mut scene = arm();
    let q = [0.0, 0.0];
    // 2 cm above the upper arm's top face.
    let above = [[0.25, 0.0, 0.25 + 0.025 + 0.02]];
    assert_eq!(scene.points_on_robot(&above, &q, 0.01), [false]);
    assert_eq!(scene.points_on_robot(&above, &q, 0.03), [true]);
    // A held part: its points go with the robot.
    scene
        .add_object(
            "part",
            &Geometry::Sphere { radius: 0.05 },
            Isometry3::translation(1.0, 0.0, 0.25),
        )
        .unwrap();
    let on_part = [[1.0, 0.0, 0.3]];
    assert_eq!(
        scene.points_on_robot(&on_part, &q, 0.005),
        [false],
        "in the world, not on the robot"
    );
    scene.attach("part", "finger", &[], &q).unwrap();
    assert_eq!(scene.points_on_robot(&on_part, &q, 0.005), [true]);
    let turned = [FRAC_PI_2, 0.0];
    assert_eq!(
        scene.points_on_robot(&[[0.0, 1.0, 0.3]], &turned, 0.005),
        [true]
    );
    // Filtered points then join the scene without blocking the arm.
    let (points, _) = reading(q);
    let kept = scene.without_robot_points(&points, &q, 0.01);
    scene
        .add_points("seen", &kept, 0.02, Isometry3::identity())
        .unwrap();
    assert!(scene.is_free(&q), "{:?}", scene.first_contact(&q));
    let mut unfiltered = arm();
    unfiltered
        .add_points("seen", &points, 0.02, Isometry3::identity())
        .unwrap();
    assert!(!unfiltered.is_free(&q));
}

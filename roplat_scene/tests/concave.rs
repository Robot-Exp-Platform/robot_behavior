//! A concave object keeps its hollow: an arm reaches into a pigeonhole that
//! the convex hull of the same mesh would fill.

use std::{fmt::Write as _, path::PathBuf};

use nalgebra::Isometry3;
use roplat_description::{Geometry, RobotModel};
use roplat_scene::{CollisionCheck, PlanningScene, SceneError};

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

/// Straight out: forearm and finger inside the pigeonhole's opening.
const REACHING_IN: [f64; 2] = [0.0, 0.0];
/// Turned 0.12 rad: the finger meets the pigeonhole's side wall.
const AGAINST_THE_WALL: [f64; 2] = [0.12, 0.0];
/// Turned well away.
const AWAY: [f64; 2] = [1.5, 0.0];

fn scene() -> PlanningScene<2> {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    PlanningScene::new(model, ["shoulder", "elbow"], |name: &str| {
        PathBuf::from(name)
    })
    .unwrap()
}

/// A box open towards the robot (-x) with 2 cm walls: outside x 0.73–1.07,
/// |y| ≤ 0.12, z 0.13–0.37; hollow to x = 1.05, |y| ≤ 0.10, z 0.15–0.35.
/// One closed surface of 14 quads.
fn pigeonhole() -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
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
    // Corner (front/back, -y/+y, low/high) of the outside and of the hollow.
    let o = |a: u32, b: u32, c: u32| 4 * a + 2 * b + c;
    let i = |a: u32, b: u32, c: u32| 8 + o(a, b, c);
    let quads = [
        // Outside: back, both sides, bottom, top.
        [o(1, 0, 0), o(1, 1, 0), o(1, 1, 1), o(1, 0, 1)],
        [o(0, 0, 0), o(1, 0, 0), o(1, 0, 1), o(0, 0, 1)],
        [o(0, 1, 0), o(1, 1, 0), o(1, 1, 1), o(0, 1, 1)],
        [o(0, 0, 0), o(1, 0, 0), o(1, 1, 0), o(0, 1, 0)],
        [o(0, 0, 1), o(1, 0, 1), o(1, 1, 1), o(0, 1, 1)],
        // The front frame around the opening.
        [o(0, 0, 0), o(0, 1, 0), i(0, 1, 0), i(0, 0, 0)],
        [o(0, 0, 1), o(0, 1, 1), i(0, 1, 1), i(0, 0, 1)],
        [o(0, 0, 0), o(0, 0, 1), i(0, 0, 1), i(0, 0, 0)],
        [o(0, 1, 0), o(0, 1, 1), i(0, 1, 1), i(0, 1, 0)],
        // The hollow: back, both sides, bottom, top.
        [i(1, 0, 0), i(1, 1, 0), i(1, 1, 1), i(1, 0, 1)],
        [i(0, 0, 0), i(1, 0, 0), i(1, 0, 1), i(0, 0, 1)],
        [i(0, 1, 0), i(1, 1, 0), i(1, 1, 1), i(0, 1, 1)],
        [i(0, 0, 0), i(1, 0, 0), i(1, 1, 0), i(0, 1, 0)],
        [i(0, 0, 1), i(1, 0, 1), i(1, 1, 1), i(0, 1, 1)],
    ];
    let triangles = quads
        .iter()
        .flat_map(|[a, b, c, d]| [[*a, *b, *c], [*a, *c, *d]])
        .collect();
    (vertices, triangles)
}

/// The pigeonhole as an OBJ file with quad faces, for the file path.
fn pigeonhole_obj() -> Geometry {
    let (vertices, triangles) = pigeonhole();
    let mut text = String::from("# pigeonhole\n");
    for [x, y, z] in &vertices {
        writeln!(text, "v {x} {y} {z}").unwrap();
    }
    // Rejoin each pair of triangles into the quad it came from.
    for pair in triangles.chunks(2) {
        let ([a, b, c], [_, _, d]) = (pair[0], pair[1]);
        writeln!(text, "f {} {}/1 {}//1 {}/1/1", a + 1, b + 1, c + 1, d + 1).unwrap();
    }
    let path = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("roplat_scene_pigeonhole.obj");
    std::fs::write(&path, text).unwrap();
    Geometry::Mesh { filename: path.to_str().unwrap().into(), scale: [1.0; 3] }
}

#[test]
fn the_decomposed_pigeonhole_leaves_its_hollow_free() {
    let mut scene = scene();
    let (vertices, triangles) = pigeonhole();
    let parts = scene
        .add_trimesh("pigeonhole", &vertices, &triangles, Isometry3::identity())
        .unwrap();
    // At least the back, two sides, bottom and top.
    assert!(parts >= 5, "{parts} parts");
    assert!(
        scene.is_free(&REACHING_IN),
        "{:?}",
        scene.first_contact(&REACHING_IN)
    );
    assert!(!scene.is_free(&AGAINST_THE_WALL));
    assert!(scene.is_free(&AWAY));
    // Moved 10 cm further out the arm still reaches in; moved 14 cm sideways
    // the straight forearm meets a side wall.
    scene
        .move_object("pigeonhole", Isometry3::translation(0.1, 0.0, 0.0))
        .unwrap();
    assert!(scene.is_free(&REACHING_IN));
    scene
        .move_object("pigeonhole", Isometry3::translation(0.0, 0.14, 0.0))
        .unwrap();
    assert!(!scene.is_free(&REACHING_IN));
}

#[test]
fn a_mesh_file_decomposes_where_its_hull_fills_the_hollow() {
    let geometry = pigeonhole_obj();
    let mut hull = scene();
    hull.add_object("pigeonhole", &geometry, Isometry3::identity())
        .unwrap();
    assert!(!hull.is_free(&REACHING_IN), "the hull fills the hollow");

    let mut parts = scene();
    let count = parts
        .add_concave_object("pigeonhole", &geometry, Isometry3::identity())
        .unwrap();
    assert!(count >= 5, "{count} parts");
    assert!(
        parts.is_free(&REACHING_IN),
        "{:?}",
        parts.first_contact(&REACHING_IN)
    );
    assert!(!parts.is_free(&AGAINST_THE_WALL));

    // Scaled to half size and placed nearer, the hollow is too shallow: the
    // finger meets its back wall.
    let Geometry::Mesh { filename, .. } = &geometry else {
        unreachable!()
    };
    let half = Geometry::Mesh { filename: filename.clone(), scale: [0.5; 3] };
    let mut small = scene();
    small
        .add_concave_object("small", &half, Isometry3::translation(0.4, 0.0, 0.125))
        .unwrap();
    assert!(!small.is_free(&REACHING_IN));
}

#[test]
fn other_shapes_and_bad_triangles() {
    let mut scene = scene();
    // Primitive shapes are placed as they are: one part.
    let block = Geometry::Box { size: [0.1, 0.1, 0.1] };
    assert_eq!(
        scene
            .add_concave_object("block", &block, Isometry3::translation(2.0, 0.0, 0.0))
            .unwrap(),
        1
    );
    let (vertices, triangles) = pigeonhole();
    assert!(matches!(
        scene.add_trimesh("block", &vertices, &triangles, Isometry3::identity()),
        Err(SceneError::NameTaken(_))
    ));
    assert!(matches!(
        scene.add_trimesh("empty", &vertices, &[], Isometry3::identity()),
        Err(SceneError::InvalidGeometry(_))
    ));
    assert!(matches!(
        scene.add_trimesh("beyond", &vertices, &[[0, 1, 16]], Isometry3::identity()),
        Err(SceneError::InvalidGeometry(_))
    ));
    let mut nan = vertices.clone();
    nan[3][1] = f64::NAN;
    assert!(matches!(
        scene.add_trimesh("nan", &nan, &triangles, Isometry3::identity()),
        Err(SceneError::InvalidGeometry(_))
    ));
    let missing = Geometry::Mesh { filename: "no/such/mesh.obj".into(), scale: [1.0; 3] };
    assert!(matches!(
        scene.add_concave_object("missing", &missing, Isometry3::identity()),
        Err(SceneError::Mesh { .. })
    ));
    assert_eq!(scene.object_names().collect::<Vec<_>>(), ["block"]);
}

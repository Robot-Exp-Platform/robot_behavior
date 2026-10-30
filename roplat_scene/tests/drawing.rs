//! The scene as triangle meshes: one per body, on the bodies' surfaces,
//! facing out, and following the arm.

use std::path::PathBuf;

use nalgebra::{Isometry3, Vector3};
use roplat_description::{Geometry, RobotModel};
use roplat_scene::{BodyKind, BodyMesh, PlanningScene};

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

fn scene() -> PlanningScene<2> {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    PlanningScene::new(model, ["shoulder", "elbow"], |name: &str| {
        PathBuf::from(name)
    })
    .unwrap()
}

/// Volume enclosed by a mesh, positive when its triangles face out.
fn volume(mesh: &BodyMesh) -> f64 {
    mesh.triangles
        .iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| Vector3::from(mesh.vertices[i as usize]));
            a.dot(&b.cross(&c)) / 6.0
        })
        .sum()
}

fn find<'a>(meshes: &'a [BodyMesh], name: &str) -> &'a BodyMesh {
    meshes.iter().find(|m| m.name == name).unwrap()
}

#[test]
fn each_body_is_one_closed_mesh_facing_out() {
    let mut scene = scene();
    scene
        .add_object(
            "crate",
            &Geometry::Box { size: [0.1, 0.2, 0.3] },
            Isometry3::translation(1.0, 0.0, 0.15),
        )
        .unwrap();
    scene
        .add_object(
            "ball",
            &Geometry::Sphere { radius: 0.05 },
            Isometry3::translation(0.0, 1.0, 0.05),
        )
        .unwrap();
    scene
        .add_object(
            "post",
            &Geometry::Capsule { radius: 0.02, length: 0.3 },
            Isometry3::translation(-1.0, 0.0, 0.2),
        )
        .unwrap();
    scene
        .add_points(
            "seen",
            &[[0.5, -0.5, 0.05], [0.52, -0.5, 0.05]],
            0.02,
            Isometry3::identity(),
        )
        .unwrap();
    let meshes = scene.meshes(&[0.0, 0.0]);
    let names: Vec<(&str, BodyKind)> = meshes.iter().map(|m| (m.name.as_str(), m.kind)).collect();
    assert_eq!(
        names,
        [
            ("base", BodyKind::Link),
            ("upper", BodyKind::Link),
            ("fore", BodyKind::Link),
            ("finger", BodyKind::Link),
            ("crate", BodyKind::Object),
            ("ball", BodyKind::Object),
            ("post", BodyKind::Object),
            ("seen", BodyKind::Object),
        ]
    );
    // A box keeps its faces: 8 corners on its surface, its exact volume.
    let crate_mesh = find(&meshes, "crate");
    assert_eq!(crate_mesh.vertices.len(), 8);
    for v in &crate_mesh.vertices {
        assert!(((v[0] - 1.0).abs() - 0.05).abs() < 1e-12 && (v[1].abs() - 0.1).abs() < 1e-12);
    }
    assert!((volume(crate_mesh) - 0.1 * 0.2 * 0.3).abs() < 1e-12);
    // Round shapes are faceted inside their surface, close to their volume.
    let ball = find(&meshes, "ball");
    let exact = 4.0 / 3.0 * std::f64::consts::PI * 0.05f64.powi(3);
    assert!(
        volume(ball) > 0.95 * exact && volume(ball) <= exact * (1.0 + 1e-9),
        "{}",
        volume(ball)
    );
    for v in &ball.vertices {
        let r = (Vector3::from(*v) - Vector3::new(0.0, 1.0, 0.05)).norm();
        assert!((r - 0.05).abs() < 1e-9, "{r}");
    }
    assert!(volume(find(&meshes, "post")) > 0.0);
    // Two voxels: two cubes of 2 cm.
    assert!((volume(find(&meshes, "seen")) - 2.0 * 0.02f64.powi(3)).abs() < 1e-12);
    // The cylinder forearm lies along x from 0.5 to 0.9 m.
    let fore = find(&meshes, "fore");
    let xs = fore.vertices.iter().map(|v| v[0]);
    let (lo, hi) = xs.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| {
        (lo.min(x), hi.max(x))
    });
    assert!(
        (lo - 0.5).abs() < 1e-9 && (hi - 0.9).abs() < 1e-9,
        "{lo} {hi}"
    );
    let exact = std::f64::consts::PI * 0.03f64.powi(2) * 0.4;
    assert!(volume(fore) > 0.95 * exact, "{}", volume(fore));
}

#[test]
fn meshes_follow_the_arm_and_what_it_carries() {
    let mut scene = scene();
    scene
        .add_object(
            "part",
            &Geometry::Box { size: [0.04; 3] },
            Isometry3::translation(0.95, 0.0, 0.25),
        )
        .unwrap();
    scene.attach("part", "finger", &[], &[0.0, 0.0]).unwrap();
    let turned = scene.meshes(&[std::f64::consts::FRAC_PI_2, 0.0]);
    let part = find(&turned, "part");
    assert_eq!(part.kind, BodyKind::Attached);
    let centre = part
        .vertices
        .iter()
        .map(|v| Vector3::from(*v))
        .sum::<Vector3<f64>>()
        / 8.0;
    // Turned a quarter about the shoulder: along y now.
    assert!(
        (centre - Vector3::new(0.0, 0.95, 0.25)).norm() < 1e-9,
        "{centre}"
    );
    let finger = find(&turned, "finger");
    let tip = finger
        .vertices
        .iter()
        .map(|v| v[1])
        .fold(f64::NEG_INFINITY, f64::max);
    assert!((tip - 0.95).abs() < 1e-9, "{tip}");
}

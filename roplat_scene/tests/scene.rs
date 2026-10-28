use std::{f64::consts::FRAC_PI_2, path::PathBuf};

use nalgebra::Isometry3;
use roplat_description::{Geometry, RobotModel};
use roplat_scene::{CollisionCheck, Contact, PlanningScene, SceneError};
use roplat_trajectory::{PlannerOptions, plan_joint_path};

/// A planar two-link arm 0.25 m above a 0.2 m base block: a 0.5 m box upper
/// arm, a 0.4 m cylinder forearm, a flange without geometry and a sphere
/// finger at 0.43 m on the forearm (0.93 m from the shoulder when straight).
const ARM: &str = r#"<robot name="toy">
  <link name="base">
    <collision><origin xyz="0 0 0.1"/><geometry>BASE</geometry></collision>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="upper"/>
    <origin xyz="0 0 0.25"/><axis xyz="0 0 1"/>
    <limit lower="-3.1" upper="3.1" effort="10" velocity="1"/>
  </joint>
  <link name="upper">
    <collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.05 0.05"/></geometry></collision>
  </link>
  <joint name="elbow" type="revolute">
    <parent link="upper"/><child link="fore"/>
    <origin xyz="0.5 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.1" upper="3.1" effort="10" velocity="1"/>
  </joint>
  <link name="fore">
    <collision>
      <origin xyz="0.2 0 0" rpy="0 1.5707963267948966 0"/>
      <geometry><cylinder radius="0.03" length="0.4"/></geometry>
    </collision>
  </link>
  <joint name="wrist" type="fixed">
    <parent link="fore"/><child link="flange"/><origin xyz="0.4 0 0"/>
  </joint>
  <link name="flange"/>
  <joint name="slide" type="prismatic">
    <parent link="flange"/><child link="finger"/>
    <origin xyz="0.03 0 0"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="0.05" effort="10" velocity="1"/>
  </joint>
  <link name="finger">
    <collision><geometry><sphere radius="0.02"/></geometry></collision>
  </link>
</robot>"#;

const BOX_BASE: &str = r#"<box size="0.2 0.2 0.2"/>"#;

fn arm_with_base(base: &str) -> RobotModel {
    RobotModel::from_urdf_str(&ARM.replace("BASE", base)).unwrap()
}

/// Mesh names as written.
fn as_path(name: &str) -> PathBuf {
    PathBuf::from(name)
}

fn scene() -> PlanningScene<2> {
    PlanningScene::new(arm_with_base(BOX_BASE), ["shoulder", "elbow"], as_path).unwrap()
}

fn cube(edge: f64) -> Geometry {
    Geometry::Box { size: [edge; 3] }
}

fn at(x: f64, y: f64, z: f64) -> Isometry3<f64> {
    Isometry3::translation(x, y, z)
}

fn pairs(contacts: &[Contact]) -> Vec<(&str, &str)> {
    contacts
        .iter()
        .map(|contact| (contact.first.as_str(), contact.second.as_str()))
        .collect()
}

/// Folded back, the finger lies inside the upper arm.
const FOLDED: [f64; 2] = [0.0, 3.1];

#[test]
fn links_joined_through_links_without_geometry_are_adjacent() {
    let scene = scene();
    let allowed: Vec<_> = scene.allowed().pairs().collect();
    assert_eq!(
        allowed,
        [("base", "upper"), ("finger", "fore"), ("fore", "upper")]
    );
    assert!(scene.allowed().is_allowed("fore", "finger"));
    assert!(!scene.allowed().is_allowed("base", "fore"));
}

#[test]
fn a_folded_arm_reports_its_own_links_unless_allowed() {
    let mut scene = scene();
    assert_eq!(scene.first_contact(&[0.0, 0.0]), None);
    let contacts = scene.contacts(&FOLDED);
    assert_eq!(pairs(&contacts), [("upper", "finger")]);
    assert!(contacts[0].distance < 0.0, "{contacts:?}");
    assert!(!scene.is_free(&FOLDED));

    scene.allowed_mut().allow("finger", "upper");
    assert!(scene.is_free(&FOLDED));
    assert!(scene.allowed_mut().forbid("upper", "finger"));
    assert!(!scene.allowed_mut().forbid("upper", "finger"));
    assert!(!scene.is_free(&FOLDED));
}

#[test]
fn objects_block_the_arm_and_padding_keeps_a_margin() {
    let mut scene = scene();
    // 0.02 m above the forearm, whose top is at 0.28 m.
    scene
        .add_object("block", &cube(0.1), at(0.7, 0.0, 0.35))
        .unwrap();
    let straight = [0.0, 0.0];
    assert!(scene.is_free(&straight));
    let nearest = scene.clearance(&straight).unwrap();
    assert_eq!(
        (nearest.first.as_str(), nearest.second.as_str()),
        ("fore", "block")
    );
    assert!((nearest.distance - 0.02).abs() < 1e-6, "{nearest:?}");

    scene.set_padding(0.03).unwrap();
    let contact = scene.first_contact(&straight).unwrap();
    assert_eq!(
        (contact.first.as_str(), contact.second.as_str()),
        ("fore", "block")
    );
    assert!((contact.distance - 0.02).abs() < 1e-6, "{contact:?}");
    scene.set_padding(0.0).unwrap();

    scene.move_object("block", at(0.7, 0.0, 0.32)).unwrap();
    let contact = scene.first_contact(&straight).unwrap();
    assert!((contact.distance + 0.01).abs() < 1e-6, "{contact:?}");
    // Swung away from it.
    assert!(scene.is_free(&[1.0, 0.0]));

    scene.remove_object("block").unwrap();
    assert!(scene.is_free(&straight));
    assert_eq!(scene.object_names().count(), 0);
}

#[test]
fn objects_in_the_world_may_overlap_each_other() {
    let mut scene = scene();
    scene
        .add_object("a", &cube(0.1), at(0.0, 1.0, 0.0))
        .unwrap();
    scene
        .add_object("b", &cube(0.1), at(0.0, 1.05, 0.0))
        .unwrap();
    assert!(scene.is_free(&[0.0, 0.0]));
}

#[test]
fn an_attached_object_moves_with_its_link_and_may_touch_listed_links() {
    let mut scene = scene();
    let straight = [0.0, 0.0];
    let turned = [FRAC_PI_2, 0.0];
    // Against the finger, whose surface reaches 0.95 m.
    scene
        .add_object("cube", &cube(0.04), at(0.96, 0.0, 0.25))
        .unwrap();
    // Where the cube will be carried, 0.02 m beyond the finger.
    scene
        .add_object("post", &cube(0.04), at(0.0, 0.99, 0.25))
        .unwrap();
    assert_eq!(pairs(&scene.contacts(&straight)), [("finger", "cube")]);

    scene
        .attach("cube", "flange", &["finger"], &straight)
        .unwrap();
    assert!(scene.is_free(&straight));
    let carried = scene.object_pose("cube", &turned).unwrap();
    assert!((carried.translation.vector - at(0.0, 0.96, 0.25).translation.vector).norm() < 1e-12);
    assert_eq!(pairs(&scene.contacts(&turned)), [("cube", "post")]);
    assert!(matches!(
        scene.move_object("cube", at(0.0, 0.0, 0.0)),
        Err(SceneError::Attached(_))
    ));
    assert!(matches!(
        scene.attach("cube", "flange", &[], &turned),
        Err(SceneError::Attached(_))
    ));

    scene.detach("cube", &turned).unwrap();
    // Both in the world now, and the finger may no longer touch the cube.
    assert_eq!(pairs(&scene.contacts(&turned)), [("finger", "cube")]);
    assert!(matches!(
        scene.detach("cube", &turned),
        Err(SceneError::NotAttached(_))
    ));
    let left = scene.object_pose("cube", &straight).unwrap();
    assert!((left.translation.vector - carried.translation.vector).norm() < 1e-12);
}

#[test]
fn joints_outside_the_group_keep_their_set_position() {
    let mut scene = scene();
    scene
        .add_object("cube", &cube(0.04), at(1.0, 0.0, 0.25))
        .unwrap();
    assert!(scene.is_free(&[0.0, 0.0]));
    // The finger slides 0.04 m out, into the cube.
    scene.set_joint("slide", 0.04).unwrap();
    assert_eq!(pairs(&scene.contacts(&[0.0, 0.0])), [("finger", "cube")]);
    let finger = scene.link_pose("finger", &[0.0, 0.0]).unwrap();
    assert!((finger.translation.x - 0.97).abs() < 1e-12);
}

#[test]
fn the_base_places_the_robot_in_the_world() {
    let mut scene = scene();
    scene
        .add_object("block", &cube(0.1), at(0.7, 0.0, 0.25))
        .unwrap();
    assert!(!scene.is_free(&[0.0, 0.0]));
    scene.set_base(at(0.0, 0.0, 0.5));
    assert!(scene.is_free(&[0.0, 0.0]));
    assert_eq!(scene.base(), at(0.0, 0.0, 0.5));
}

#[test]
fn invalid_input_is_refused() {
    let model = arm_with_base(BOX_BASE);
    let new = |group| PlanningScene::new(model.clone(), group, as_path).map(|_| ());
    assert!(matches!(
        new(["shoulder", "knee"]),
        Err(SceneError::UnknownJoint(_))
    ));
    assert!(matches!(
        new(["shoulder", "wrist"]),
        Err(SceneError::FixedJoint(_))
    ));
    assert!(matches!(
        new(["elbow", "elbow"]),
        Err(SceneError::RepeatedJoint(_))
    ));

    let mut scene = scene();
    assert!(matches!(
        scene.set_joint("elbow", 0.0),
        Err(SceneError::GroupJoint(_))
    ));
    assert!(matches!(
        scene.set_joint("wrist", 0.0),
        Err(SceneError::FixedJoint(_))
    ));
    assert!(matches!(
        scene.set_joint("knee", 0.0),
        Err(SceneError::UnknownJoint(_))
    ));
    assert!(matches!(
        scene.set_padding(-0.01),
        Err(SceneError::InvalidPadding(_))
    ));
    assert!(matches!(
        scene.set_padding(f64::NAN),
        Err(SceneError::InvalidPadding(_))
    ));

    let origin = at(0.0, 0.0, 0.0);
    assert!(matches!(
        scene.add_object("upper", &cube(0.1), origin),
        Err(SceneError::NameTaken(_))
    ));
    scene.add_object("cube", &cube(0.1), origin).unwrap();
    assert!(matches!(
        scene.add_object("cube", &cube(0.1), origin),
        Err(SceneError::NameTaken(_))
    ));
    assert!(matches!(
        scene.add_object("flat", &Geometry::Box { size: [0.1, 0.0, 0.1] }, origin),
        Err(SceneError::InvalidGeometry(_))
    ));
    let mesh = Geometry::Mesh { filename: "cube.dae".into(), scale: [1.0; 3] };
    assert!(matches!(
        scene.add_object("dae", &mesh, origin),
        Err(SceneError::Mesh { .. })
    ));
    let q = [0.0, 0.0];
    assert!(matches!(
        scene.attach("cube", "hand", &[], &q),
        Err(SceneError::UnknownLink(_))
    ));
    assert!(matches!(
        scene.attach("cube", "flange", &["thumb"], &q),
        Err(SceneError::UnknownLink(_))
    ));
    assert!(matches!(
        scene.attach("ball", "flange", &[], &q),
        Err(SceneError::UnknownObject(_))
    ));
    assert!(matches!(
        scene.remove_object("ball"),
        Err(SceneError::UnknownObject(_))
    ));
    assert!(matches!(
        scene.link_pose("hand", &q),
        Err(SceneError::UnknownLink(_))
    ));
}

/// The eight corners of a cube of edge 0.1 m centred on the origin, as
/// triangles: two per face.
fn cube_triangles() -> Vec<[[f64; 3]; 3]> {
    let h = 0.05;
    let corner = |i: usize| {
        [
            if i & 1 == 0 { -h } else { h },
            if i & 2 == 0 { -h } else { h },
            if i & 4 == 0 { -h } else { h },
        ]
    };
    let faces = [
        [0, 1, 3, 2],
        [4, 6, 7, 5],
        [0, 4, 5, 1],
        [2, 3, 7, 6],
        [0, 2, 6, 4],
        [1, 5, 7, 3],
    ];
    faces
        .iter()
        .flat_map(|f| {
            [
                [corner(f[0]), corner(f[1]), corner(f[2])],
                [corner(f[0]), corner(f[2]), corner(f[3])],
            ]
        })
        .collect()
}

fn write_meshes() -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("roplat_scene_meshes");
    std::fs::create_dir_all(&dir).unwrap();
    let triangles = cube_triangles();

    let mut obj = String::from("# cube\no cube\n");
    for triangle in &triangles {
        for [x, y, z] in triangle {
            obj += &format!("v {x} {y} {z}\nvn 0 0 1\nvt 0 0\n");
        }
    }
    // An interior vertex does not change the hull.
    obj += "v 0.01 0.0 0.0\nf 1/1/1 2/2/2 3/3/3\n";
    std::fs::write(dir.join("cube.obj"), obj).unwrap();

    let mut ascii = String::from("solid cube\n");
    for triangle in &triangles {
        ascii += "facet normal 0 0 0\nouter loop\n";
        for [x, y, z] in triangle {
            ascii += &format!("vertex {x} {y} {z}\n");
        }
        ascii += "endloop\nendfacet\n";
    }
    ascii += "endsolid cube\n";
    std::fs::write(dir.join("cube_ascii.STL"), ascii).unwrap();

    let mut binary = vec![0u8; 80];
    binary.extend((triangles.len() as u32).to_le_bytes());
    for triangle in &triangles {
        binary.extend([0u8; 12]);
        for vertex in triangle {
            for value in vertex {
                binary.extend((*value as f32).to_le_bytes());
            }
        }
        binary.extend([0u8; 2]);
    }
    std::fs::write(dir.join("cube_binary.stl"), binary).unwrap();

    std::fs::write(dir.join("flat.obj"), "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\n").unwrap();
    dir
}

#[test]
fn meshes_become_their_convex_hulls() {
    let dir = write_meshes();
    let mut scene = scene();
    let straight = [0.0, 0.0];
    let mesh = |name: &str, scale| Geometry::Mesh {
        filename: dir.join(name).to_string_lossy().into_owned(),
        scale,
    };
    // Each hull ends 0.05 m beyond the finger's surface at 0.95 m.
    for (name, scale, centre) in [
        ("cube.obj", [2.0, 1.0, 1.0], 1.1),
        ("cube_ascii.STL", [1.0; 3], 1.05),
        ("cube_binary.stl", [1.0; 3], 1.05),
    ] {
        scene
            .add_object(name, &mesh(name, scale), at(centre, 0.0, 0.25))
            .unwrap();
        let nearest = scene.clearance(&straight).unwrap();
        assert_eq!(
            (nearest.first.as_str(), nearest.second.as_str()),
            ("finger", name)
        );
        assert!(
            (nearest.distance - 0.05).abs() < 1e-6,
            "{name}: {nearest:?}"
        );
        scene.remove_object(name).unwrap();
    }
    for name in ["flat.obj", "missing.obj"] {
        assert!(matches!(
            scene.add_object(name, &mesh(name, [1.0; 3]), at(0.0, 0.0, 0.0)),
            Err(SceneError::Mesh { .. })
        ));
    }

    // A link's mesh, resolved through the scene's mesh path: the same base
    // block as the box.
    let model = arm_with_base(r#"<mesh filename="package://toy/cube.obj" scale="2 2 2"/>"#);
    let mut resolved = Vec::new();
    let meshed = PlanningScene::new(model, ["shoulder", "elbow"], |name| {
        resolved.push(name.to_owned());
        dir.join(name.trim_start_matches("package://toy/"))
    })
    .unwrap();
    assert_eq!(resolved, ["package://toy/cube.obj"]);
    let boxed = self::scene();
    for q in [[0.0, 0.0], FOLDED, [0.4, -2.5], [-1.2, 2.9]] {
        let (a, b) = (meshed.clearance(&q).unwrap(), boxed.clearance(&q).unwrap());
        assert_eq!((&a.first, &a.second), (&b.first, &b.second), "{q:?}");
        assert!((a.distance - b.distance).abs() < 1e-6, "{q:?}: {a:?} {b:?}");
    }
}

#[test]
fn the_planner_routes_around_an_object() {
    let mut scene = scene();
    // In the straight arm's sweep, beyond the reach of the folded arm.
    scene
        .add_object(
            "pillar",
            &Geometry::Box { size: [0.1, 0.1, 0.2] },
            at(0.75, 0.0, 0.25),
        )
        .unwrap();
    let (start, goal) = ([-1.0, 0.0], [1.0, 0.0]);
    assert!(scene.is_free(&start) && scene.is_free(&goal));
    assert!(!scene.is_free(&[0.0, 0.0]));

    scene.set_padding(0.01).unwrap();
    let path = plan_joint_path(
        start,
        goal,
        [-3.1; 2],
        [3.1; 2],
        |q| scene.is_free(q),
        PlannerOptions { resolution: 0.01, ..Default::default() },
    )
    .unwrap();
    assert!(path.len() > 2, "{path:?}");

    scene.set_padding(0.0).unwrap();
    for pair in path.windows(2) {
        let [a, b] = [pair[0], pair[1]];
        let steps = ((b[0] - a[0]).abs().max((b[1] - a[1]).abs()) / 1e-3).ceil() as usize;
        for k in 0..=steps {
            let t = k as f64 / steps.max(1) as f64;
            let q = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
            assert!(scene.is_free(&q), "{q:?}: {:?}", scene.first_contact(&q));
        }
    }
}

//! From a depth image to a planned pick: a camera sees a six-joint arm
//! leaning over a table with a block and a can on it. The arm's own points
//! are removed, the table and the objects are segmented and fitted, the
//! objects enter the planning scene, and a pick of the block is planned with
//! every segment checked whole.

use std::path::PathBuf;

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use parry3d_f64::{
    query::Ray,
    shape::{SharedShape, TriMesh},
};
use roplat_description::{Geometry, RobotModel};
use roplat_grasp::{GraspOptions, PickOptions, grasps, plan_pick_motion};
use roplat_kinematics::{IkSearch, chain_ik_solutions};
use roplat_msgs::{CameraInfo, geometry::Pose};
use roplat_perception::{TabletopOptions, segment_tabletop};
use roplat_scene::{BodyKind, CollisionCheck, EndEffector, PlanningScene, points::depth_to_points};

/// Base column 0.3 m, upper arm 0.4 m, forearm 0.35 m, a spherical wrist, a
/// palm and two fingers 0.06 m apart (as in roplat_grasp's `pick.rs`).
const ARM: &str = r#"<robot name="arm6">
  <link name="base"><collision><origin xyz="0 0 0.15"/><geometry><cylinder radius="0.06" length="0.3"/></geometry></collision></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.3"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="l1"/>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><axis xyz="0 1 0"/><limit lower="-2.5" upper="2.5" effort="1" velocity="1"/></joint>
  <link name="l2"><collision><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.36"/></geometry></collision></link>
  <joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/><origin xyz="0 0 0.4"/><axis xyz="0 1 0"/><limit lower="-2.8" upper="2.8" effort="1" velocity="1"/></joint>
  <link name="l3"><collision><origin xyz="0 0 0.175"/><geometry><box size="0.04 0.04 0.3"/></geometry></collision></link>
  <joint name="j4" type="revolute"><parent link="l3"/><child link="l4"/><origin xyz="0 0 0.35"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="l4"/>
  <joint name="j5" type="revolute"><parent link="l4"/><child link="l5"/><axis xyz="0 1 0"/><limit lower="-2.5" upper="2.5" effort="1" velocity="1"/></joint>
  <link name="l5"/>
  <joint name="j6" type="revolute"><parent link="l5"/><child link="flange"/><origin xyz="0 0 0.08"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="flange"/>
  <joint name="palm_joint" type="fixed"><parent link="flange"/><child link="palm"/></joint>
  <link name="palm"><collision><origin xyz="0 0 0.02"/><geometry><box size="0.04 0.1 0.04"/></geometry></collision></link>
  <joint name="left_joint" type="fixed"><parent link="palm"/><child link="left"/><origin xyz="0 0.03 0.07"/></joint>
  <link name="left"><collision><geometry><box size="0.02 0.01 0.06"/></geometry></collision></link>
  <joint name="right_joint" type="fixed"><parent link="palm"/><child link="right"/><origin xyz="0 -0.03 0.07"/></joint>
  <link name="right"><collision><geometry><box size="0.02 0.01 0.06"/></geometry></collision></link>
</robot>"#;

const JOINTS: [&str; 6] = ["j1", "j2", "j3", "j4", "j5", "j6"];
const LOWER: [f64; 6] = [-3.0, -2.5, -2.8, -3.0, -2.5, -3.0];
const UPPER: [f64; 6] = [3.0, 2.5, 2.8, 3.0, 2.5, 3.0];
/// Leaning over the table: the forearm runs about 0.25 m above its top.
const LEANING: [f64; 6] = [0.0, 1.0, 0.8, 0.0, 0.0, 0.0];
const TABLE_TOP: f64 = 0.2;
const BLOCK: [f64; 3] = [0.04, 0.04, 0.08];
const BLOCK_AT: [f64; 2] = [0.45, 0.12];
const CAN_AT: [f64; 2] = [0.55, -0.15];

fn table() -> (Geometry, Isometry3<f64>) {
    (
        Geometry::Box { size: [0.5, 0.8, 0.04] },
        Isometry3::translation(0.55, 0.0, TABLE_TOP - 0.02),
    )
}

/// The arm and the table, as the robot knows them before it looks.
fn scene() -> PlanningScene<6> {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    let mut scene = PlanningScene::new(model, JOINTS, |name: &str| PathBuf::from(name)).unwrap();
    let (geometry, pose) = table();
    scene.add_object("table", &geometry, pose).unwrap();
    scene
}

/// A camera at `eye` looking at `target`: optical `z` forward, `x` right,
/// `y` down.
fn look_at(eye: Vector3<f64>, target: Vector3<f64>) -> Isometry3<f64> {
    let z = (target - eye).normalize();
    let x = z.cross(&Vector3::z()).normalize();
    let y = z.cross(&x);
    let rotation = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[x, y, z]));
    Isometry3::from_parts(
        Translation3::from(eye),
        UnitQuaternion::from_rotation_matrix(&rotation),
    )
}

/// What the camera sees: the table, the block and the can, and the arm at
/// `q` drawn from the scene's own meshes; points in the world.
fn sense(scene: &PlanningScene<6>, q: &[f64; 6]) -> Vec<[f64; 3]> {
    let (table, table_pose) = table();
    let Geometry::Box { size } = table else {
        unreachable!()
    };
    let mut shapes: Vec<(Isometry3<f64>, SharedShape)> = vec![
        (
            table_pose,
            SharedShape::cuboid(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0),
        ),
        (
            Isometry3::new(
                Vector3::new(BLOCK_AT[0], BLOCK_AT[1], TABLE_TOP + BLOCK[2] / 2.0),
                Vector3::new(0.0, 0.0, 0.3),
            ),
            SharedShape::cuboid(BLOCK[0] / 2.0, BLOCK[1] / 2.0, BLOCK[2] / 2.0),
        ),
        (
            Isometry3::new(
                Vector3::new(CAN_AT[0], CAN_AT[1], TABLE_TOP + 0.05),
                Vector3::new(std::f64::consts::FRAC_PI_2, 0.0, 0.0),
            ),
            SharedShape::cylinder(0.05, 0.03),
        ),
    ];
    for mesh in scene
        .meshes(q)
        .into_iter()
        .filter(|m| m.kind == BodyKind::Link)
    {
        let vertices = mesh.vertices.iter().map(|v| Point3::from(*v)).collect();
        let trimesh = TriMesh::new(vertices, mesh.triangles).unwrap();
        shapes.push((Isometry3::identity(), SharedShape::new(trimesh)));
    }
    let camera = look_at(
        Vector3::new(0.5, -0.65, 1.1),
        Vector3::new(0.5, 0.0, TABLE_TOP),
    );
    let (width, height, f) = (240usize, 180usize, 220.0);
    let (cx, cy) = (119.5, 89.5);
    let info = CameraInfo {
        width: width as u32,
        height: height as u32,
        k: [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
        ..Default::default()
    };
    let mut depth = Vec::with_capacity(width * height);
    for v in 0..height {
        for u in 0..width {
            let direction = Vector3::new((u as f64 - cx) / f, (v as f64 - cy) / f, 1.0);
            let ray = Ray::new(
                Point3::from(camera.translation.vector),
                camera.rotation * direction,
            );
            let hit = shapes
                .iter()
                .filter_map(|(pose, shape)| shape.cast_ray(pose, &ray, 10.0, true))
                .fold(f64::INFINITY, f64::min);
            depth.push(hit as f32);
        }
    }
    depth_to_points(&depth, &info, 0.1..=3.0, 1)
        .into_iter()
        .map(|p| (camera * Point3::from(p)).coords.into())
        .collect()
}

fn inverse(model: &RobotModel) -> impl FnMut(&Isometry3<f64>, &[f64; 6]) -> Option<[f64; 6]> {
    let chain = model.chain("base", "flange").unwrap();
    move |target, seed| {
        let search = IkSearch { restarts: 24, max_solutions: 1, ..IkSearch::default() };
        let solutions = chain_ik_solutions(
            &chain,
            &Pose::from_isometry(*target),
            seed,
            |_| true,
            search,
        )
        .ok()?;
        solutions.first().map(|q| std::array::from_fn(|i| q[i]))
    }
}

#[test]
fn a_pick_is_planned_from_what_the_camera_sees() {
    let mut scene = scene();
    assert!(
        scene.is_free(&LEANING),
        "{:?}",
        scene.first_contact(&LEANING)
    );
    let points = sense(&scene, &LEANING);

    // Unfiltered, the forearm over the table is segmented as an object too.
    let options = TabletopOptions::default();
    let unfiltered = segment_tabletop(&points, &options).unwrap();
    let mine = scene.without_robot_points(&points, &LEANING, 0.02);
    assert!(mine.len() < points.len());
    let found = segment_tabletop(&mine, &options).unwrap();
    assert!(
        unfiltered.objects.len() > found.objects.len(),
        "{} vs {}",
        unfiltered.objects.len(),
        found.objects.len()
    );
    assert_eq!(
        found.objects.len(),
        2,
        "{:#?}",
        found.objects.iter().map(|o| &o.shape).collect::<Vec<_>>()
    );
    assert!((found.table.offset - TABLE_TOP).abs() < 0.003);

    // The block and the can, where they were placed.
    let near = |at: [f64; 2]| {
        found
            .objects
            .iter()
            .position(|o| {
                let c = o.shape.pose.translation.vector;
                (c.x - at[0]).hypot(c.y - at[1]) < 0.01
            })
            .unwrap_or_else(|| panic!("nothing found at {at:?}"))
    };
    let (block, can) = (near(BLOCK_AT), near(CAN_AT));
    assert!(matches!(
        found.objects[block].shape.geometry,
        Geometry::Box { .. }
    ));
    assert!(matches!(
        found.objects[can].shape.geometry,
        Geometry::Cylinder { .. }
    ));
    for (name, k) in [("block", block), ("can", can)] {
        let shape = &found.objects[k].shape;
        scene.add_object(name, &shape.geometry, shape.pose).unwrap();
    }
    assert!(scene.is_free(&LEANING));

    // Pick the block, every segment checked whole.
    let shape = &found.objects[block].shape;
    let candidates = grasps(&shape.geometry, &shape.pose, &GraspOptions::default());
    assert!(!candidates.is_empty());
    let hand = EndEffector::new("hand", "flange", Isometry3::translation(0.0, 0.0, 0.09))
        .with_links(["palm", "left", "right"]);
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    let plan = plan_pick_motion(
        &scene,
        &hand,
        "block",
        Some("table"),
        &candidates,
        LEANING,
        LOWER,
        UPPER,
        inverse(&model),
        PickOptions::default(),
    )
    .unwrap_or_else(|e| panic!("{e}"));
    for pair in plan.to_pregrasp.windows(2) {
        assert!(scene.motion_is_free(&pair[0], &pair[1]), "{pair:?}");
    }
    // The tool ends at the grasp, on the block the camera saw.
    let tool = hand.pose(&scene, &plan.grasp_configuration()).unwrap();
    let centre = shape.pose.translation.vector;
    assert!((tool.translation.vector - plan.grasp.pose.translation.vector).norm() < 1e-3);
    assert!((plan.grasp.pose.translation.vector.xy() - centre.xy()).norm() < 0.01);
}

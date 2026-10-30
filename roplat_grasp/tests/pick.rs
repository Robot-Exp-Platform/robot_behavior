//! Picking a block off a table with a six-joint arm and a parallel gripper.

use std::path::PathBuf;

use nalgebra::{Isometry3, Vector3};
use roplat_description::{Geometry, RobotModel};
use roplat_grasp::{GraspOptions, PickFailure, PickOptions, grasps, plan_pick};
use roplat_kinematics::{IkSearch, chain_ik_solutions};
use roplat_msgs::geometry::Pose;
use roplat_scene::{CollisionCheck, EndEffector, NamedPoses, PlanningScene};

/// Base column 0.3 m, upper arm 0.4 m, forearm 0.35 m, a spherical wrist,
/// a 0.04 m palm on the flange and two fingers 0.06 m apart.
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
/// Table top at z = 0.2; the block's centre 0.04 above it.
const BLOCK: [f64; 3] = [0.04, 0.04, 0.08];

fn model() -> RobotModel {
    RobotModel::from_urdf_str(ARM).unwrap()
}

/// The tool centre point between the fingertips, 0.09 m out of the flange.
fn hand() -> EndEffector {
    EndEffector::new("hand", "flange", Isometry3::translation(0.0, 0.0, 0.09))
        .with_links(["palm", "left", "right"])
}

fn scene(block_at: Vector3<f64>) -> PlanningScene<6> {
    let mut scene = PlanningScene::new(model(), JOINTS, |name: &str| PathBuf::from(name)).unwrap();
    scene
        .add_object(
            "table",
            &Geometry::Box { size: [0.5, 0.8, 0.04] },
            Isometry3::translation(0.55, 0.0, 0.18),
        )
        .unwrap();
    scene
        .add_object(
            "block",
            &Geometry::Box { size: BLOCK },
            Isometry3::translation(block_at.x, block_at.y, block_at.z),
        )
        .unwrap();
    scene
}

/// Inverse kinematics of the flange: several random starts, the solution
/// nearest the seed.
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

fn start() -> [f64; 6] {
    // Upright, the hand pointing up.
    [0.0; 6]
}

#[test]
fn picks_a_block_from_above_and_every_waypoint_is_free() {
    let at = Vector3::new(0.45, 0.1, 0.24);
    let scene = scene(at);
    let hand = hand();
    let candidates = grasps(
        &Geometry::Box { size: BLOCK },
        &Isometry3::translation(at.x, at.y, at.z),
        &GraspOptions::default(),
    );
    assert!(!candidates.is_empty());
    let model = model();
    let plan = plan_pick(
        &scene,
        &hand,
        "block",
        Some("table"),
        &candidates,
        start(),
        LOWER,
        UPPER,
        inverse(&model),
        PickOptions::default(),
    )
    .unwrap_or_else(|e| panic!("{e}"));

    // A top grasp is preferred and reachable here.
    assert!((plan.grasp.approach() - (-Vector3::z())).norm() < 1e-9);

    // The paths join up.
    assert_eq!(plan.to_pregrasp.first(), Some(&start()));
    assert_eq!(plan.to_pregrasp.last(), plan.approach.first());
    assert_eq!(plan.approach.last(), plan.lift.first());

    // The tool ends at the grasp, then 0.1 m above it.
    let at_grasp = hand.pose(&scene, &plan.grasp_configuration()).unwrap();
    assert!((at_grasp.translation.vector - plan.grasp.pose.translation.vector).norm() < 1e-4);
    let lifted = hand.pose(&scene, plan.lift.last().unwrap()).unwrap();
    assert!(
        (lifted.translation.vector - at_grasp.translation.vector - Vector3::new(0.0, 0.0, 0.1))
            .norm()
            < 1e-4
    );

    // Before the grasp nothing touches; the approach itself is checked with
    // the fingers allowed to touch the block (they do not here: the block is
    // 0.04 wide and the fingers 0.05 apart inside).
    for q in plan.to_pregrasp.iter().chain(&plan.approach) {
        assert!(
            scene.is_free(q),
            "contact at {q:?}: {:?}",
            scene.first_contact(q)
        );
    }
    // Lifting with the block attached clears the table once it is up.
    let mut carrying = scene.clone();
    carrying
        .attach(
            "block",
            "flange",
            &["palm", "left", "right"],
            &plan.grasp_configuration(),
        )
        .unwrap();
    assert!(carrying.is_free(plan.lift.last().unwrap()));
}

#[test]
fn reports_why_each_candidate_failed() {
    // Far out of reach: every candidate's pre-grasp is unreachable.
    let at = Vector3::new(1.6, 0.0, 0.24);
    let scene = scene(at);
    let candidates = grasps(
        &Geometry::Box { size: BLOCK },
        &Isometry3::translation(at.x, at.y, at.z),
        &GraspOptions::default(),
    );
    let model = model();
    let err = plan_pick(
        &scene,
        &hand(),
        "block",
        Some("table"),
        &candidates,
        start(),
        LOWER,
        UPPER,
        inverse(&model),
        PickOptions::default(),
    )
    .unwrap_err();
    assert_eq!(err.attempts.len(), candidates.len());
    assert!(
        err.attempts
            .iter()
            .all(|(_, why)| *why == PickFailure::PregraspUnreachable)
    );

    // No candidates at all.
    let err = plan_pick(
        &scene,
        &hand(),
        "block",
        None,
        &[],
        start(),
        LOWER,
        UPPER,
        inverse(&model),
        PickOptions::default(),
    )
    .unwrap_err();
    assert!(err.attempts.is_empty());
    assert_eq!(err.to_string(), "no grasp candidates");
}

#[test]
fn named_poses_check_against_the_scene() {
    let scene = scene(Vector3::new(0.45, 0.1, 0.24));
    let mut poses = NamedPoses::<6>::new();
    poses.insert("up", start());
    // Tipped forward through the table top: in collision.
    poses.insert("slumped", [0.0, 2.0, 0.0, 0.0, 0.0, 0.0]);
    let free: Vec<_> = poses.free_in(&scene).collect();
    assert_eq!(free, ["up"]);
    assert_eq!(
        poses
            .nearest(&[0.0, 1.5, 0.0, 0.0, 0.0, 0.0])
            .map(|(n, _)| n),
        Some("slumped")
    );
}

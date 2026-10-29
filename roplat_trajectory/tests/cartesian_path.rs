//! Straight tip paths on a UR5-like arm read from URDF, solved by `chain_ik`.

use nalgebra as na;
use roplat_behavior::Pose;
use roplat_description::{Chain, RobotModel};
use roplat_kinematics::{CommonStop, chain_ik};
use roplat_trajectory::{CartesianPathOptions, CartesianStop, plan_cartesian_path};

/// UR5 standard DH rows `(d, a, alpha)`, all offsets zero.
const UR5: [(f64, f64, f64); 6] = [
    (0.089159, 0.0, std::f64::consts::FRAC_PI_2),
    (0.0, -0.425, 0.0),
    (0.0, -0.39225, 0.0),
    (0.10915, 0.0, std::f64::consts::FRAC_PI_2),
    (0.09465, 0.0, -std::f64::consts::FRAC_PI_2),
    (0.0823, 0.0, 0.0),
];

const HOME: [f64; 6] = [0.3, -1.0, 1.4, -1.97, -1.57, 0.2];

fn origin(t: na::Isometry3<f64>) -> String {
    let p = t.translation.vector;
    let (r, pi, y) = t.rotation.euler_angles();
    format!(
        r#"<origin xyz="{} {} {}" rpy="{r} {pi} {y}"/>"#,
        p.x, p.y, p.z
    )
}

fn chain() -> Chain {
    let rx = |a: f64| na::Isometry3::rotation(na::Vector3::x() * a);
    let mut links = String::from(r#"<link name="l0"/>"#);
    let mut joints = String::new();
    let mut after = na::Isometry3::identity();
    for (i, (d, a, alpha)) in UR5.into_iter().enumerate() {
        let before = after * na::Isometry3::translation(0.0, 0.0, d);
        links += &format!(r#"<link name="l{}"/>"#, i + 1);
        joints += &format!(
            r#"<joint name="j{}" type="revolute"><parent link="l{i}"/><child link="l{}"/>{}<axis xyz="0 0 1"/>
               <limit lower="-6.28" upper="6.28" velocity="2.5" effort="150"/></joint>"#,
            i + 1,
            i + 1,
            origin(before)
        );
        after = na::Isometry3::translation(a, 0.0, 0.0) * rx(alpha);
    }
    links += r#"<link name="tool"/>"#;
    joints += &format!(
        r#"<joint name="tool_mount" type="fixed"><parent link="l6"/><child link="tool"/>{}</joint>"#,
        origin(after)
    );
    let urdf = format!(r#"<robot name="ur5">{links}{joints}</robot>"#);
    RobotModel::from_urdf_str(&urdf)
        .unwrap()
        .chain("l0", "tool")
        .unwrap()
}

fn fk(chain: &Chain, q: &[f64; 6]) -> na::Isometry3<f64> {
    chain.forward_kinematics_iso(q).unwrap()
}

fn ik(chain: &Chain) -> impl FnMut(&na::Isometry3<f64>, &[f64; 6]) -> Option<[f64; 6]> + '_ {
    let stop = CommonStop { pos_tol: 1e-9, rot_tol: 1e-9, max_iters: 200, step_clip: 0.2 };
    move |pose, seed| {
        let s = chain_ik(chain, seed, &Pose::from_isometry(*pose), 1e-3, stop).ok()?;
        s.converged.then(|| s.q.try_into().unwrap())
    }
}

fn shifted(pose: na::Isometry3<f64>, by: [f64; 3]) -> na::Isometry3<f64> {
    na::Translation3::new(by[0], by[1], by[2]) * pose
}

/// Largest distance of the tip from the line through `a` and `b`, and the
/// largest orientation change from `a`.
fn deviation(
    chain: &Chain,
    waypoints: &[[f64; 6]],
    a: &na::Isometry3<f64>,
    b: &na::Isometry3<f64>,
) -> (f64, f64) {
    let d = (b.translation.vector - a.translation.vector).normalize();
    waypoints.iter().fold((0.0, 0.0), |(line, turn), q| {
        let t = fk(chain, q);
        let r = t.translation.vector - a.translation.vector;
        (
            f64::max(line, (r - d * r.dot(&d)).norm()),
            f64::max(turn, t.rotation.angle_to(&a.rotation)),
        )
    })
}

#[test]
fn follows_a_straight_line() {
    let chain = chain();
    let start = fk(&chain, &HOME);
    let goal = shifted(start, [0.1, -0.15, 0.2]);
    let options = CartesianPathOptions::default();
    let path = plan_cartesian_path(
        HOME,
        &[goal],
        |q| fk(&chain, q),
        ik(&chain),
        |_| true,
        options,
    );
    assert_eq!(path.stop, None);
    assert_eq!(path.fraction, 1.0);
    // 0.27 m at 1 cm per step.
    assert_eq!(path.waypoints.len(), 1 + 27);
    let (line, turn) = deviation(&chain, &path.waypoints, &start, &goal);
    assert!(line < 1e-8 && turn < 1e-8, "{line} {turn}");
    let end = fk(&chain, path.waypoints.last().unwrap());
    assert!((end.translation.vector - goal.translation.vector).norm() < 1e-8);
    for pair in path.waypoints.windows(2) {
        let step = (fk(&chain, &pair[1]).translation.vector
            - fk(&chain, &pair[0]).translation.vector)
            .norm();
        assert!(step <= options.max_step + 1e-9, "{step}");
    }
}

#[test]
fn passes_through_each_pose_and_bounds_rotation_steps() {
    let chain = chain();
    let start = fk(&chain, &HOME);
    let turn = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), 0.6);
    let a = shifted(start, [0.1, 0.0, 0.0]);
    let b = na::Isometry3::from_parts(shifted(a, [0.0, 0.1, 0.0]).translation, turn * a.rotation);
    let c = shifted(b, [-0.1, 0.0, 0.0]);
    let options = CartesianPathOptions::default();
    let path = plan_cartesian_path(
        HOME,
        &[a, b, c],
        |q| fk(&chain, q),
        ik(&chain),
        |_| true,
        options,
    );
    assert_eq!(path.stop, None);
    // 10 steps each for a and c; b is limited by 0.6 rad at 0.05 per step.
    assert_eq!(path.waypoints.len(), 1 + 10 + 12 + 10);
    for (index, pose) in [(10, a), (22, b), (32, c)] {
        let t = fk(&chain, &path.waypoints[index]);
        assert!((t.translation.vector - pose.translation.vector).norm() < 1e-8);
        assert!(t.rotation.angle_to(&pose.rotation) < 1e-8);
    }
    for pair in path.waypoints[10..=22].windows(2) {
        let step = fk(&chain, &pair[1])
            .rotation
            .angle_to(&fk(&chain, &pair[0]).rotation);
        assert!(step <= options.max_rotation_step + 1e-9, "{step}");
    }
}

#[test]
fn stops_before_an_obstacle_between_waypoints() {
    let chain = chain();
    let start = fk(&chain, &HOME);
    let x0 = start.translation.x;
    let goal = shifted(start, [0.2, 0.0, 0.0]);
    // A 4 mm slab 0.103 m ahead: no 5 cm waypoint lands in it, only the
    // segment between the second and third does.
    let slab = |q: &[f64; 6]| {
        let x = fk(&chain, q).translation.x - x0;
        !(0.103..0.107).contains(&x)
    };
    let options = CartesianPathOptions { max_step: 0.05, resolution: 0.002, ..Default::default() };
    let path = plan_cartesian_path(HOME, &[goal], |q| fk(&chain, q), ik(&chain), slab, options);
    assert_eq!(path.stop, Some(CartesianStop::Invalid));
    assert_eq!(path.waypoints.len(), 3);
    assert_eq!(path.fraction, 0.5);
}

#[test]
fn stops_where_the_arm_cannot_reach() {
    let chain = chain();
    let start = fk(&chain, &HOME);
    let reach = start.translation.vector.normalize();
    let goal = shifted(start, (reach * 1.0).into());
    let path = plan_cartesian_path(
        HOME,
        &[goal],
        |q| fk(&chain, q),
        ik(&chain),
        |_| true,
        Default::default(),
    );
    assert_eq!(path.stop, Some(CartesianStop::Unreachable));
    assert!(
        path.fraction > 0.0 && path.fraction < 1.0,
        "{}",
        path.fraction
    );
    let (line, _) = deviation(&chain, &path.waypoints, &start, &goal);
    assert!(line < 1e-8);
}

#[test]
fn stops_at_a_joint_jump_or_an_invalid_start() {
    let chain = chain();
    let start = fk(&chain, &HOME);
    let goal = shifted(start, [0.0, 0.0, 0.1]);
    let mut solve = ik(&chain);
    let mut calls = 0;
    // A solver that switches to the other wrist branch at the fifth pose.
    let flipping = |pose: &na::Isometry3<f64>, seed: &[f64; 6]| {
        calls += 1;
        let mut q = solve(pose, seed)?;
        if calls >= 5 {
            q[5] += 1.0;
        }
        Some(q)
    };
    let path = plan_cartesian_path(
        HOME,
        &[goal],
        |q| fk(&chain, q),
        flipping,
        |_| true,
        Default::default(),
    );
    assert_eq!(path.stop, Some(CartesianStop::JointJump));
    assert_eq!(path.waypoints.len(), 5);

    let nan = |_: &na::Isometry3<f64>, _: &[f64; 6]| Some([f64::NAN; 6]);
    let path = plan_cartesian_path(
        HOME,
        &[goal],
        |q| fk(&chain, q),
        nan,
        |_| true,
        Default::default(),
    );
    assert_eq!(path.stop, Some(CartesianStop::Unreachable));

    let path = plan_cartesian_path(
        HOME,
        &[goal],
        |q| fk(&chain, q),
        ik(&chain),
        |_| false,
        Default::default(),
    );
    assert_eq!(path.stop, Some(CartesianStop::InvalidStart));
    assert_eq!((path.waypoints, path.fraction), (vec![HOME], 0.0));
}

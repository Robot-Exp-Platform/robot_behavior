//! A URDF chain written from a DH table gives the same cache as the table.

use nalgebra as na;
use roplat_behavior::{ForwardKinematics, JacobianModel};
use roplat_description::RobotModel;
use roplat_kinematics::{
    ArmKineCache, ChainError, CommonStop, DhParam, Link, UrdfKinematics, chain_ik,
    dh_chain_discrepancy,
};

/// Standard DH rows `(theta, d, r, alpha)`, as in `kine_cache.rs`.
const DH6: [(f64, f64, f64, f64); 6] = [
    (0.0, 0.12, 0.0, std::f64::consts::FRAC_PI_2),
    (0.3, 0.0, 0.36, 0.0),
    (-0.2, 0.0, 0.30, 0.0),
    (0.0, 0.11, 0.0, std::f64::consts::FRAC_PI_2),
    (0.1, 0.09, 0.0, -std::f64::consts::FRAC_PI_2),
    (0.0, 0.08, 0.0, 0.0),
];

fn origin(t: na::Isometry3<f64>) -> String {
    let p = t.translation.vector;
    let (r, pi, y) = t.rotation.euler_angles();
    format!(
        r#"<origin xyz="{} {} {}" rpy="{r} {pi} {y}"/>"#,
        p.x, p.y, p.z
    )
}

/// Joint `i` rotates about z after `Tx(r_{i-1}) Rx(α_{i-1}) Rz(θ_i) Tz(d_i)`;
/// the last `Tx(r) Rx(α)` is a fixed flange joint.
fn urdf() -> String {
    let rz = |a: f64| na::Isometry3::rotation(na::Vector3::z() * a);
    let rx = |a: f64| na::Isometry3::rotation(na::Vector3::x() * a);
    let mut links = String::from(r#"<link name="l0"/>"#);
    let mut joints = String::new();
    let mut after = na::Isometry3::identity();
    for (i, (theta, d, r, alpha)) in DH6.into_iter().enumerate() {
        let before = after * rz(theta) * na::Isometry3::translation(0.0, 0.0, d);
        links += &format!(r#"<link name="l{}"/>"#, i + 1);
        joints += &format!(
            r#"<joint name="j{}" type="revolute"><parent link="l{i}"/><child link="l{}"/>{}<axis xyz="0 0 1"/>
               <limit lower="-3" upper="3" velocity="1" effort="1"/></joint>"#,
            i + 1,
            i + 1,
            origin(before)
        );
        after = na::Isometry3::translation(r, 0.0, 0.0) * rx(alpha);
    }
    links += r#"<link name="flange"/>"#;
    joints += &format!(
        r#"<joint name="flange_mount" type="fixed"><parent link="l6"/><child link="flange"/>{}</joint>"#,
        origin(after)
    );
    format!(r#"<robot name="dh6">{links}{joints}</robot>"#)
}

fn close(a: &na::Isometry3<f64>, b: &na::Isometry3<f64>) -> bool {
    (a.translation.vector - b.translation.vector).norm() < 1e-9
        && a.rotation.angle_to(&b.rotation) < 1e-9
}

#[test]
fn urdf_chain_matches_the_dh_table() {
    let dh = DH6.map(|(theta, d, r, alpha)| roplat_kinematics::DhParam::DH { theta, d, r, alpha });
    let model = RobotModel::from_urdf_str(&urdf()).unwrap();
    let chain = model.chain("l0", "flange").unwrap();

    for q in [[0.0; 6], [0.4, -0.7, 1.1, 0.2, -0.5, 0.9]] {
        let expected = ArmKineCache::build(&dh, &q, &[0.0; 6]);
        let cache = ArmKineCache::from_chain(&chain, &q, &[0.0; 6]).unwrap();
        // A URDF link frame stops before the DH row's `Tx(r) Rx(α)`, which the
        // next joint's origin carries; the base and the flange coincide.
        for (link, (_, _, r, alpha)) in DH6.into_iter().enumerate().take(5) {
            let offset = na::Isometry3::translation(r, 0.0, 0.0)
                * na::Isometry3::rotation(na::Vector3::x() * alpha);
            assert!(
                close(
                    &(cache.link_pose(Link(link + 1)) * offset),
                    &expected.link_pose(Link(link + 1))
                ),
                "link {} at {q:?}",
                link + 1
            );
        }
        assert!(close(
            &cache.link_pose(Link(0)),
            &expected.link_pose(Link(0))
        ));
        assert!(close(
            &cache.end_effector_pose(),
            &expected.end_effector_pose()
        ));
        assert!((cache.jacobian() - expected.jacobian()).norm() < 1e-9);
        assert!((cache.jacobian() - chain.jacobian(&q).unwrap()).norm() < 1e-9);
        assert!(close(
            &cache.end_effector_pose(),
            &chain.forward_kinematics_iso(&q).unwrap()
        ));
    }
}

#[test]
fn urdf_kinematics_gives_the_dh_model_maps() {
    let dh = DH6.map(|(theta, d, r, alpha)| DhParam::DH { theta, d, r, alpha });
    let model = RobotModel::from_urdf_str(&urdf()).unwrap();
    let kinematics = UrdfKinematics::<6>::new(model.chain("l0", "flange").unwrap()).unwrap();
    let q = [0.4, -0.7, 1.1, 0.2, -0.5, 0.9];
    let expected = ArmKineCache::build(&dh, &q, &[0.0; 6]);
    let pose = kinematics.forward_kinematics(q).unwrap().quat();
    assert!(close(&pose, &expected.end_effector_pose()));
    assert!((kinematics.jacobian(q).unwrap() - expected.jacobian()).norm() < 1e-9);
    let limits = kinematics.joint_limits();
    assert_eq!(
        (limits.q_min[0], limits.q_max[0], limits.dq_max[0]),
        (-3.0, 3.0, 1.0)
    );
    assert!(UrdfKinematics::<5>::new(model.chain("l0", "flange").unwrap()).is_err());
}

#[test]
fn rejects_wrong_length_and_prismatic_joints() {
    let model = RobotModel::from_urdf_str(&urdf()).unwrap();
    let chain = model.chain("l0", "flange").unwrap();
    assert!(matches!(
        ArmKineCache::<5>::from_chain(&chain, &[0.0; 5], &[0.0; 5]),
        Err(ChainError::Description(_))
    ));

    let slider = r#"<robot name="s"><link name="a"/><link name="b"/>
        <joint name="slide" type="prismatic"><parent link="a"/><child link="b"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="1" velocity="1" effort="1"/></joint></robot>"#;
    let model = RobotModel::from_urdf_str(slider).unwrap();
    let chain = model.chain("a", "b").unwrap();
    assert!(matches!(
        ArmKineCache::<1>::from_chain(&chain, &[0.0], &[0.0]),
        Err(ChainError::Prismatic(name)) if name == "slide"
    ));
}

fn stop() -> CommonStop {
    CommonStop { pos_tol: 1e-8, rot_tol: 1e-8, max_iters: 500, step_clip: 0.2 }
}

#[test]
fn chain_ik_recovers_poses_from_perturbed_seeds() {
    let model = RobotModel::from_urdf_str(&urdf()).unwrap();
    let chain = model.chain("l0", "flange").unwrap();
    let cases = [
        [0.4, -0.7, 1.1, 0.2, -0.5, 0.9],
        [-1.2, 0.3, -0.8, 1.5, 0.7, -2.0],
        [2.1, 1.0, 0.4, -1.1, 1.3, 0.2],
    ];
    for q_true in cases {
        let target = chain.forward_kinematics(&q_true).unwrap();
        let q0: Vec<f64> = q_true
            .iter()
            .enumerate()
            .map(|(i, q)| q + 0.3 * (-1f64).powi(i as i32))
            .collect();
        let solution = chain_ik(&chain, &q0, &target, 1e-3, stop()).unwrap();
        assert!(solution.converged, "{q_true:?}: {solution:?}");
        let reached = chain.forward_kinematics_iso(&solution.q).unwrap();
        let target = target.quat();
        assert!((reached.translation.vector - target.translation.vector).norm() < 1e-8);
        assert!(reached.rotation.angle_to(&target.rotation) < 1e-8);
        assert!(solution.q.iter().all(|q| (-3.0..=3.0).contains(q)));
    }
}

#[test]
fn chain_ik_handles_prismatic_joints_and_respects_limits() {
    let arm = r#"<robot name="lift_arm"><link name="w"/><link name="a"/><link name="b"/><link name="c"/>
        <joint name="lift" type="prismatic"><parent link="w"/><child link="a"/><axis xyz="0 0 1"/>
          <limit lower="0" upper="0.5" velocity="1" effort="1"/></joint>
        <joint name="yaw" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/>
          <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
        <joint name="pitch" type="revolute"><parent link="b"/><child link="c"/><origin xyz="0.3 0 0"/><axis xyz="0 1 0"/>
          <limit lower="-1" upper="1" velocity="1" effort="1"/></joint></robot>"#;
    let model = RobotModel::from_urdf_str(arm).unwrap();
    let chain = model.chain("w", "c").unwrap();

    let target = chain.forward_kinematics(&[0.3, 0.5, -0.4]).unwrap();
    let solution = chain_ik(&chain, &[0.1, 0.0, 0.0], &target, 1e-3, stop()).unwrap();
    assert!(solution.converged, "{solution:?}");
    assert!((solution.q[0] - 0.3).abs() < 1e-6);

    // Reaching it needs the lift at 0.8 m, beyond its 0.5 m limit.
    let beyond = chain.forward_kinematics(&[0.5, 0.5, -0.4]).unwrap();
    let mut beyond = beyond;
    beyond.position.z += 0.3;
    let solution = chain_ik(&chain, &[0.1, 0.0, 0.0], &beyond, 1e-3, stop()).unwrap();
    assert!(!solution.converged);
    assert!((solution.position_error - 0.3).abs() < 1e-3, "{solution:?}");
    assert!(solution.q[0] <= 0.5 && solution.q[0] >= 0.0);

    assert!(chain_ik(&chain, &[0.0], &target, 1e-3, stop()).is_err());
}

#[test]
fn dh_chain_discrepancy_tells_same_and_different_arms_apart() {
    let model = RobotModel::from_urdf_str(&urdf()).unwrap();
    let chain = model.chain("l0", "l6").unwrap();
    let dh = DH6.map(|(theta, d, r, alpha)| DhParam::DH { theta, d, r, alpha });
    let samples: Vec<[f64; 6]> = (0..5)
        .map(|k| std::array::from_fn(|i| ((i + 1) as f64 * (k as f64 + 0.7)).sin()))
        .collect();
    // The chain stops before the flange: same arm, different tool frame.
    assert!(dh_chain_discrepancy(&dh, &chain, &samples).unwrap() < 1e-9);

    // The third joint turning the other way is a different arm.
    let mut flipped = dh;
    if let DhParam::DH { alpha, .. } = &mut flipped[1] {
        *alpha = std::f64::consts::PI;
    }
    assert!(dh_chain_discrepancy(&flipped, &chain, &samples).unwrap() > 0.1);
}

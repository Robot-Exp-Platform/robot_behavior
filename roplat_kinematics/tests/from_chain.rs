//! A URDF chain written from a DH table gives the same cache as the table.

use nalgebra as na;
use roplat_description::RobotModel;
use roplat_kinematics::{ArmKineCache, ChainError, Link};

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

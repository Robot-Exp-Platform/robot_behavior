//! `ArmKineCache` stores links `1..=N` and treats the base as the identity;
//! these checks pin its observable behaviour without depending on that layout.

use nalgebra as na;
use roplat_kinematics::{ArmKineCache, DhParam, Link};

const DH6: [DhParam; 6] = [
    DhParam::DH {
        theta: 0.0,
        d: 0.12,
        r: 0.0,
        alpha: std::f64::consts::FRAC_PI_2,
    },
    DhParam::DH { theta: 0.3, d: 0.0, r: 0.36, alpha: 0.0 },
    DhParam::DH { theta: -0.2, d: 0.0, r: 0.30, alpha: 0.0 },
    DhParam::DH {
        theta: 0.0,
        d: 0.11,
        r: 0.0,
        alpha: std::f64::consts::FRAC_PI_2,
    },
    DhParam::DH {
        theta: 0.1,
        d: 0.09,
        r: 0.0,
        alpha: -std::f64::consts::FRAC_PI_2,
    },
    DhParam::DH { theta: 0.0, d: 0.08, r: 0.0, alpha: 0.0 },
];

const MDH4: [DhParam; 4] = [
    DhParam::MDH { alpha: 0.0, a: 0.0, theta: 0.0, d: 0.33 },
    DhParam::MDH {
        alpha: -std::f64::consts::FRAC_PI_2,
        a: 0.0,
        theta: 0.0,
        d: 0.0,
    },
    DhParam::MDH {
        alpha: std::f64::consts::FRAC_PI_2,
        a: 0.0,
        theta: 0.0,
        d: 0.31,
    },
    DhParam::MDH {
        alpha: std::f64::consts::FRAC_PI_2,
        a: 0.08,
        theta: 0.0,
        d: 0.0,
    },
];

fn close(a: &na::Isometry3<f64>, b: &na::Isometry3<f64>) -> bool {
    (a.translation.vector - b.translation.vector).norm() < 1e-12
        && a.rotation.angle_to(&b.rotation) < 1e-12
}

fn check_chain<const N: usize>(dh: &[DhParam; N], q: [f64; N]) {
    let cache = ArmKineCache::build(dh, &q, &[0.0; N]);

    // Link 0 is the base; link N is the end effector.
    assert!(close(&cache.link_pose(Link(0)), &na::Isometry3::identity()));
    assert!(close(&cache.link_pose(Link(N)), &cache.end_effector_pose()));

    // Relative poses agree with the absolute ones, including the base.
    for a in 0..=N {
        for b in 0..=N {
            let expected = cache.link_pose(Link(b)).inv_mul(&cache.link_pose(Link(a)));
            assert!(close(&cache.relative_pose(Link(a), Link(b)), &expected));
        }
    }

    // The Jacobian matches central differences of the end-effector pose.
    let j = cache.jacobian();
    let h = 1e-6;
    for i in 0..N {
        let (mut qp, mut qm) = (q, q);
        qp[i] += h;
        qm[i] -= h;
        let tp = ArmKineCache::build(dh, &qp, &[0.0; N]).end_effector_pose();
        let tm = ArmKineCache::build(dh, &qm, &[0.0; N]).end_effector_pose();
        let dv = (tp.translation.vector - tm.translation.vector) / (2.0 * h);
        let dw = (tp.rotation * tm.rotation.inverse()).scaled_axis() / (2.0 * h);
        let col = j.column(i);
        assert!(
            (col.fixed_rows::<3>(0) - dv).norm() < 1e-6,
            "linear column {i}"
        );
        assert!(
            (col.fixed_rows::<3>(3) - dw).norm() < 1e-6,
            "angular column {i}"
        );
    }

    // The Jacobian at link N is the end-effector Jacobian; at the base it is zero.
    assert_eq!(cache.jacobian_at_link(N), j);
    assert_eq!(cache.jacobian_at_link(0), na::SMatrix::<f64, 6, N>::zeros());
}

#[test]
fn dh_chain_frames_and_jacobian_are_consistent() {
    check_chain(&DH6, [0.4, -0.7, 1.1, 0.2, -0.5, 0.9]);
    check_chain(&DH6, [0.0; 6]);
}

#[test]
fn mdh_chain_frames_and_jacobian_are_consistent() {
    check_chain(&MDH4, [0.3, -1.2, 0.6, 0.8]);
}

#[test]
fn empty_chain_is_the_base() {
    let cache = ArmKineCache::<0>::build(&[], &[], &[]);
    assert!(close(
        &cache.end_effector_pose(),
        &na::Isometry3::identity()
    ));
    assert!(close(&cache.link_pose(Link(0)), &na::Isometry3::identity()));
}

//! Inverse dynamics against closed forms.

use roplat_description::RobotModel;

const G: f64 = 9.81;

/// A 2 kg rod pivoting about y, centre of mass 0.3 m along x, Iyy 0.05 about
/// the centre, and a massless tool on a fixed joint.
const PENDULUM: &str = r#"<robot name="pendulum"><link name="base"/>
  <link name="rod"><inertial><origin xyz="0.3 0 0"/><mass value="2"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/></inertial></link>
  <link name="tool"/>
  <joint name="pivot" type="revolute"><parent link="base"/><child link="rod"/><origin xyz="0 0 1"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
  <joint name="mount" type="fixed"><parent link="rod"/><child link="tool"/><origin xyz="0.6 0 0"/></joint>
</robot>"#;

#[test]
fn pendulum_matches_the_closed_form() {
    let model = RobotModel::from_urdf_str(PENDULUM).unwrap();
    let (m, l, i) = (2.0, 0.3, 0.05);
    for (q, qd, qdd) in [(0.0, 0.0, 0.0), (0.7, 1.3, -2.0), (-1.1, -0.4, 3.5)] {
        let efforts = model.inverse_dynamics(|_| (q, qd, qdd), [0.0, 0.0, -G]);
        // Rotating about +y takes +x towards −z, so gravity pulls q up.
        let expected = (i + m * l * l) * qdd - m * G * l * f64::cos(q);
        assert!(
            (efforts[0] - expected).abs() < 1e-9,
            "{q} {qd} {qdd}: {efforts:?}"
        );
        assert_eq!(efforts[1], 0.0, "fixed joint");
    }
}

#[test]
fn prismatic_lift_carries_weight_and_inertia() {
    let lift = r#"<robot name="lift"><link name="base"/>
      <link name="carriage"><inertial><origin xyz="0.1 0.2 0"/><mass value="5"/>
        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
      <joint name="z" type="prismatic"><parent link="base"/><child link="carriage"/><axis xyz="0 0 1"/>
        <limit lower="0" upper="1" velocity="1" effort="1"/></joint></robot>"#;
    let model = RobotModel::from_urdf_str(lift).unwrap();
    let efforts = model.inverse_dynamics(|_| (0.4, 0.2, 1.5), [0.0, 0.0, -G]);
    assert!((efforts[0] - 5.0 * (1.5 + G)).abs() < 1e-9, "{efforts:?}");

    // Gravity along the root's x does not load a vertical slide.
    let efforts = model.inverse_dynamics(|_| (0.4, 0.2, 0.0), [-G, 0.0, 0.0]);
    assert!(efforts[0].abs() < 1e-12);
}

/// A horizontal planar 2R arm (axes along z, gravity along −z so it adds
/// nothing): the textbook closed form with Coriolis and centripetal terms.
#[test]
fn planar_two_link_matches_the_closed_form() {
    let (m1, m2, l1, c1, c2, i1, i2) = (3.0, 2.0, 0.5, 0.2, 0.15, 0.04, 0.02);
    let arm = format!(
        r#"<robot name="planar2r"><link name="base"/>
      <link name="upper"><inertial><origin xyz="{c1} 0 0"/><mass value="{m1}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="{i1}"/></inertial></link>
      <link name="fore"><inertial><origin xyz="{c2} 0 0"/><mass value="{m2}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="{i2}"/></inertial></link>
      <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/><axis xyz="0 0 1"/>
        <limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
      <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/><origin xyz="{l1} 0 0"/>
        <axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="1" effort="1"/></joint></robot>"#
    );
    let model = RobotModel::from_urdf_str(&arm).unwrap();
    let (q, qd, qdd) = ([0.3, 1.1], [0.8, -1.7], [2.0, -0.5]);
    let efforts = model.inverse_dynamics(
        |j| {
            let i = usize::from(j.name == "elbow");
            (q[i], qd[i], qdd[i])
        },
        [0.0, 0.0, -G],
    );

    let (s2, k2) = (q[1].sin(), q[1].cos());
    let m11 = i1 + i2 + m1 * c1 * c1 + m2 * (l1 * l1 + c2 * c2 + 2.0 * l1 * c2 * k2);
    let m12 = i2 + m2 * (c2 * c2 + l1 * c2 * k2);
    let m22 = i2 + m2 * c2 * c2;
    let h = m2 * l1 * c2 * s2;
    let tau1 = m11 * qdd[0] + m12 * qdd[1] - h * (2.0 * qd[0] * qd[1] + qd[1] * qd[1]);
    let tau2 = m12 * qdd[0] + m22 * qdd[1] + h * qd[0] * qd[0];
    assert!((efforts[0] - tau1).abs() < 1e-9, "{efforts:?} vs {tau1}");
    assert!((efforts[1] - tau2).abs() < 1e-9, "{efforts:?} vs {tau2}");
}

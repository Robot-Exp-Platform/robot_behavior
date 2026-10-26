use std::f64::consts::FRAC_PI_2;

use roplat_description::{DescriptionError, Geometry, JointKind, RobotModel};

/// Planar 2R arm (link lengths 0.4 and 0.3) on a 0.1 m prismatic lift, with a
/// fixed 0.05 m tool offset.
const ARM: &str = r#"
<robot name="planar">
  <link name="world"/>
  <link name="base">
    <inertial><mass value="2.0"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/></inertial>
    <collision><geometry><cylinder radius="0.05" length="0.1"/></geometry></collision>
  </link>
  <link name="upper"><visual><origin xyz="0.2 0 0"/><geometry><box size="0.4 0.02 0.02"/></geometry></visual></link>
  <link name="fore"><visual><geometry><mesh filename="package://planar/fore.stl" scale="0.001 0.001 0.001"/></geometry></visual></link>
  <link name="tool"/>
  <joint name="lift" type="prismatic">
    <parent link="world"/><child link="base"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="0.1" velocity="0.2" effort="100"/>
  </joint>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="upper"/><axis xyz="0 0 1"/>
    <limit lower="-2.0" upper="2.0" velocity="1.5" effort="40"/>
  </joint>
  <joint name="elbow" type="continuous">
    <parent link="upper"/><child link="fore"/><origin xyz="0.4 0 0"/><axis xyz="0 0 1"/>
    <limit velocity="2.0" effort="20"/>
  </joint>
  <joint name="tool_mount" type="fixed">
    <parent link="fore"/><child link="tool"/><origin xyz="0.35 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"#;

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-12
}

#[test]
fn reads_tree_inertia_and_geometry() {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    assert_eq!(model.name, "planar");
    assert_eq!(model.root().name, "world");
    let base = model.link("base").unwrap();
    let inertial = base.inertial.unwrap();
    assert_eq!(inertial.mass, 2.0);
    assert_eq!(inertial.inertia, [0.1, 0.0, 0.0, 0.1, 0.0, 0.2]);
    assert_eq!(
        base.collisions[0].geometry,
        Geometry::Cylinder { radius: 0.05, length: 0.1 }
    );
    assert!(model.link("tool").unwrap().inertial.is_none());
    match &model.link("fore").unwrap().visuals[0].geometry {
        Geometry::Mesh { filename, scale } => {
            assert_eq!(filename, "package://planar/fore.stl");
            assert_eq!(*scale, [0.001; 3]);
        }
        other => panic!("unexpected {other:?}"),
    }
    assert_eq!(model.joint("elbow").unwrap().kind, JointKind::Continuous);
    assert_eq!(model.joint("tool_mount").unwrap().limit, None);
}

#[test]
fn forward_kinematics_follows_the_chain() {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    let chain = model.chain("world", "tool").unwrap();
    assert_eq!(chain.dof(), 3);
    assert_eq!(chain.joint_names(), ["lift", "shoulder", "elbow"]);

    let p = chain
        .forward_kinematics(&[0.05, 0.0, 0.0])
        .unwrap()
        .position;
    assert!(close(p.x, 0.75) && close(p.y, 0.0) && close(p.z, 0.05));

    let p = chain
        .forward_kinematics(&[0.0, FRAC_PI_2, 0.0])
        .unwrap()
        .position;
    assert!(close(p.x, 0.0) && close(p.y, 0.75));

    let p = chain
        .forward_kinematics(&[0.0, 0.0, FRAC_PI_2])
        .unwrap()
        .position;
    assert!(close(p.x, 0.4) && close(p.y, 0.35));

    // A sub-chain starts at its own base.
    let sub = model.chain("upper", "tool").unwrap();
    assert_eq!(sub.joint_names(), ["elbow"]);
    assert!(close(
        sub.forward_kinematics(&[0.0]).unwrap().position.x,
        0.75
    ));

    assert!(matches!(
        chain.forward_kinematics(&[0.0]),
        Err(DescriptionError::DofMismatch { expected: 3, got: 1 })
    ));
}

#[test]
fn limits_convert_to_the_shared_message() {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    let chain = model.chain("world", "tool").unwrap();
    let limits = chain.joint_limits::<3>().unwrap();
    assert_eq!(limits.q_min[..2], [0.0, -2.0]);
    assert_eq!(limits.q_max[..2], [0.1, 2.0]);
    assert_eq!(limits.q_min[2], f64::NEG_INFINITY);
    assert_eq!(limits.dq_max, [0.2, 1.5, 2.0]);
    assert_eq!(limits.tau_max, [100.0, 40.0, 20.0]);
    assert!(limits.ddq_max.iter().all(|a| a.is_infinite()));
    assert!(chain.joint_limits::<2>().is_err());

    assert!(
        chain
            .violated_position_limits(&[0.05, 1.0, 100.0])
            .unwrap()
            .is_empty()
    );
    assert_eq!(
        chain.violated_position_limits(&[0.2, -3.0, 0.0]).unwrap(),
        ["lift", "shoulder"]
    );
}

#[test]
fn rejects_bad_trees() {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    assert!(matches!(
        model.chain("tool", "world"),
        Err(DescriptionError::NotAncestor { .. })
    ));
    assert!(matches!(
        model.chain("world", "nope"),
        Err(DescriptionError::UnknownLink(_))
    ));

    let two_roots = r#"<robot name="r"><link name="a"/><link name="b"/></robot>"#;
    assert!(matches!(
        RobotModel::from_urdf_str(two_roots),
        Err(DescriptionError::RootCount(2))
    ));

    let dangling = r#"<robot name="r"><link name="a"/>
        <joint name="j" type="fixed"><parent link="a"/><child link="ghost"/></joint></robot>"#;
    assert!(matches!(
        RobotModel::from_urdf_str(dangling),
        Err(DescriptionError::DanglingJoint { .. })
    ));
}

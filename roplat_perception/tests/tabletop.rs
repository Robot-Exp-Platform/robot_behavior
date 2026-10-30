//! A depth camera looks at a table with two boxes and a cylinder on it;
//! the objects found are the ones placed, and become grasp candidates and
//! planning-scene obstacles.

use std::{path::PathBuf, sync::OnceLock};

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use parry3d_f64::{query::Ray, shape::SharedShape};
use roplat_description::{Geometry, RobotModel};
use roplat_grasp::{GraspOptions, grasps};
use roplat_msgs::CameraInfo;
use roplat_perception::{
    IcpOptions, PlaneOptions, TabletopOptions, crop_box, fit_plane, icp, segment_tabletop,
};
use roplat_scene::{CollisionCheck, PlanningScene, points::depth_to_points};

const TABLE_TOP: f64 = 0.7;

/// What is placed on the table.
struct Truth {
    centre: [f64; 2],
    yaw: f64,
    geometry: Geometry,
}

fn truths() -> Vec<Truth> {
    vec![
        Truth {
            centre: [0.5, 0.05],
            yaw: 0.5,
            geometry: Geometry::Box { size: [0.06, 0.1, 0.15] },
        },
        Truth {
            centre: [0.75, -0.1],
            yaw: 0.0,
            geometry: Geometry::Cylinder { radius: 0.035, length: 0.12 },
        },
        Truth {
            centre: [0.42, -0.2],
            yaw: 0.0,
            geometry: Geometry::Box { size: [0.05, 0.05, 0.05] },
        },
    ]
}

/// The table (a slab whose top is at 0.7 m), the floor, and the objects,
/// as parry shapes. Cylinders in parry run along y; URDF's along z.
fn world() -> Vec<(Isometry3<f64>, SharedShape)> {
    let mut shapes = vec![
        (
            Isometry3::translation(0.6, 0.0, TABLE_TOP - 0.02),
            SharedShape::cuboid(0.4, 0.5, 0.02),
        ),
        (
            Isometry3::translation(0.6, 0.0, -0.01),
            SharedShape::cuboid(3.0, 3.0, 0.01),
        ),
    ];
    for truth in truths() {
        let height = match truth.geometry {
            Geometry::Box { size } => size[2],
            Geometry::Cylinder { length, .. } => length,
            _ => unreachable!(),
        };
        let pose = Isometry3::new(
            Vector3::new(truth.centre[0], truth.centre[1], TABLE_TOP + height / 2.0),
            Vector3::new(0.0, 0.0, truth.yaw),
        );
        shapes.push(match truth.geometry {
            Geometry::Box { size } => (
                pose,
                SharedShape::cuboid(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0),
            ),
            Geometry::Cylinder { radius, length } => (
                pose * Isometry3::rotation(Vector3::new(std::f64::consts::FRAC_PI_2, 0.0, 0.0)),
                SharedShape::cylinder(length / 2.0, radius),
            ),
            _ => unreachable!(),
        });
    }
    shapes
}

/// A camera at `eye` looking at `target`, in the optical convention:
/// `z` along the view, `x` right, `y` down.
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

/// A 320 x 240 depth image of the world by ray casting, with about a
/// millimetre of noise, and its intrinsics.
fn render(camera: &Isometry3<f64>) -> (Vec<f32>, CameraInfo) {
    let (width, height) = (320usize, 240usize);
    let (f, cx, cy) = (300.0, 159.5, 119.5);
    let info = CameraInfo {
        width: width as u32,
        height: height as u32,
        k: [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
        ..Default::default()
    };
    let shapes = world();
    let mut state = 0x853c_49e6_748f_ea9b_u64;
    let mut noise = move || {
        // The sum of four uniforms: roughly normal, 1 mm spread.
        (0..4)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                (state >> 11) as f64 / (1u64 << 53) as f64 - 0.5
            })
            .sum::<f64>()
            * 0.0017
    };
    let mut depth = Vec::with_capacity(width * height);
    for v in 0..height {
        for u in 0..width {
            // With a unit z, the time of impact is the depth.
            let direction = Vector3::new((u as f64 - cx) / f, (v as f64 - cy) / f, 1.0);
            let ray = Ray::new(
                Point3::from(camera.translation.vector),
                camera.rotation * direction,
            );
            let hit = shapes
                .iter()
                .filter_map(|(pose, shape)| shape.cast_ray(pose, &ray, 10.0, true))
                .fold(f64::INFINITY, f64::min);
            depth.push(if hit.is_finite() {
                (hit + noise()) as f32
            } else {
                f32::NAN
            });
        }
    }
    (depth, info)
}

/// The sensed points in the world, rendered once.
fn sensed() -> &'static [[f64; 3]] {
    static POINTS: OnceLock<Vec<[f64; 3]>> = OnceLock::new();
    POINTS.get_or_init(|| {
        let camera = look_at(
            Vector3::new(0.6, -0.55, 1.35),
            Vector3::new(0.6, 0.0, TABLE_TOP),
        );
        let (depth, info) = render(&camera);
        depth_to_points(&depth, &info, 0.1..=3.0, 1)
            .into_iter()
            .map(|p| (camera * Point3::from(p)).coords.into())
            .collect()
    })
}

fn yaw_of(pose: &Isometry3<f64>) -> f64 {
    let x = pose.rotation * Vector3::x();
    x.y.atan2(x.x)
}

#[test]
fn the_objects_on_the_table_are_found_and_fitted() {
    let points = sensed();
    let found = segment_tabletop(points, &TabletopOptions::default()).unwrap();
    // The table, not the floor, which lies beyond the table's edges.
    assert!(
        found.table.normal.z > (0.5f64).to_radians().cos(),
        "{:?}",
        found.table
    );
    assert!(
        (found.table.offset - TABLE_TOP).abs() < 0.002,
        "{:?}",
        found.table
    );
    assert!(found.outline.len() >= 4);
    assert_eq!(
        found.objects.len(),
        3,
        "{:#?}",
        found.objects.iter().map(|o| &o.shape).collect::<Vec<_>>()
    );

    for truth in truths() {
        let object = found
            .objects
            .iter()
            .min_by(|a, b| {
                let distance = |o: &&roplat_perception::TabletopObject| {
                    let c = o.shape.pose.translation.vector;
                    (c.x - truth.centre[0]).hypot(c.y - truth.centre[1])
                };
                distance(a).total_cmp(&distance(b))
            })
            .unwrap();
        let centre = object.shape.pose.translation.vector;
        let offset = (centre.x - truth.centre[0]).hypot(centre.y - truth.centre[1]);
        assert!(offset < 0.006, "{:?} found at {centre}", truth.geometry);
        match (&truth.geometry, &object.shape.geometry) {
            (Geometry::Box { size: placed }, Geometry::Box { size }) => {
                let mut sides = [size[0], size[1]];
                let mut truth_sides = [placed[0], placed[1]];
                sides.sort_by(f64::total_cmp);
                truth_sides.sort_by(f64::total_cmp);
                for k in 0..2 {
                    assert!(
                        (sides[k] - truth_sides[k]).abs() < 0.01,
                        "{placed:?} as {size:?}"
                    );
                }
                assert!(
                    (size[2] - placed[2]).abs() < 0.006,
                    "{placed:?} as {size:?}"
                );
                // Sides along the placed yaw, up to a quarter turn.
                let quarter = std::f64::consts::FRAC_PI_2;
                let error = (yaw_of(&object.shape.pose) - truth.yaw).rem_euclid(quarter);
                assert!(
                    error.min(quarter - error) < 3f64.to_radians(),
                    "yaw off by {error}"
                );
            }
            (
                Geometry::Cylinder { radius: r0, length: l0 },
                Geometry::Cylinder { radius, length },
            ) => {
                assert!((radius - r0).abs() < 0.005, "radius {radius}");
                assert!((length - l0).abs() < 0.006, "length {length}");
            }
            (placed, fitted) => panic!("{placed:?} fitted as {fitted:?}"),
        }
        // The bottom rests on the table.
        let bottom = centre.z
            - match object.shape.geometry {
                Geometry::Box { size } => size[2] / 2.0,
                Geometry::Cylinder { length, .. } => length / 2.0,
                _ => unreachable!(),
            };
        assert!((bottom - TABLE_TOP).abs() < 0.003, "bottom at {bottom}");
    }
}

#[test]
fn fitted_objects_feed_grasping_and_the_planning_scene() {
    let found = segment_tabletop(sensed(), &TabletopOptions::default()).unwrap();
    let options = GraspOptions { max_opening: 0.08, ..Default::default() };
    for object in &found.objects {
        let candidates = grasps(&object.shape.geometry, &object.shape.pose, &options);
        assert!(!candidates.is_empty(), "{:?}", object.shape);
        // The best candidate approaches from above.
        assert!(candidates[0].approach().z < -0.9, "{:?}", candidates[0]);
    }
    let model = RobotModel::from_urdf_str(
        r#"<robot name="post"><link name="base"><collision><origin xyz="0 0 0.4"/><geometry><box size="0.1 0.1 0.8"/></geometry></collision></link>
        <joint name="turn" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.8"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
        <link name="arm"><collision><origin xyz="0.3 0 0"/><geometry><box size="0.6 0.04 0.04"/></geometry></collision></link></robot>"#,
    )
    .unwrap();
    let mut scene = PlanningScene::new(model, ["turn"], |name: &str| PathBuf::from(name)).unwrap();
    for (k, object) in found.objects.iter().enumerate() {
        scene
            .add_object(
                &format!("object{k}"),
                &object.shape.geometry,
                object.shape.pose,
            )
            .unwrap();
    }
    // The arm sweeps 0.78 m to 0.82 m up: pointing away it is free, pointing
    // at the tall box (0.85 m) it is not.
    assert!(scene.is_free(&[3.0]));
    let tall = found
        .objects
        .iter()
        .position(|o| matches!(o.shape.geometry, Geometry::Box { size } if size[2] > 0.1))
        .unwrap();
    let contact = scene.first_contact(&[0.05f64.atan2(0.5)]).unwrap();
    assert_eq!(contact.second, format!("object{tall}"));
}

#[test]
fn icp_refines_a_model_onto_the_sensed_box() {
    let points = sensed();
    let found = segment_tabletop(points, &TabletopOptions::default()).unwrap();
    let truth = &truths()[0];
    let tall = found
        .objects
        .iter()
        .find(|o| matches!(o.shape.geometry, Geometry::Box { size } if size[2] > 0.1))
        .unwrap();
    // The box's model: its top and sides every 3 mm, centred on its frame.
    let Geometry::Box { size } = truth.geometry else {
        unreachable!()
    };
    let half = size.map(|s| s / 2.0);
    let mut model = Vec::new();
    let steps = size.map(|s| (s / 0.003) as usize);
    for i in 0..=steps[0] {
        for j in 0..=steps[1] {
            for k in 0..=steps[2] {
                let p = [
                    -half[0] + size[0] * i as f64 / steps[0] as f64,
                    -half[1] + size[1] * j as f64 / steps[1] as f64,
                    -half[2] + size[2] * k as f64 / steps[2] as f64,
                ];
                let on_side = i == 0 || i == steps[0] || j == 0 || j == steps[1] || k == steps[2];
                if on_side {
                    model.push(p);
                }
            }
        }
    }
    // The sensed points near the box, and a guess 1 cm and 0.1 rad off.
    let truth_pose = Isometry3::new(
        Vector3::new(truth.centre[0], truth.centre[1], TABLE_TOP + half[2]),
        Vector3::new(0.0, 0.0, truth.yaw),
    );
    let near = crop_box(points, &truth_pose, [0.1, 0.1, 0.1]);
    let near: Vec<[f64; 3]> = near
        .into_iter()
        .filter(|p| p[2] > TABLE_TOP + 0.005)
        .collect();
    let guess =
        Isometry3::new(Vector3::new(0.01, -0.005, 0.0), Vector3::new(0.0, 0.0, 0.1)) * truth_pose;
    // The camera sees two sides and the top, the model has all: match each
    // sensed point onto the model, and invert.
    let options = IcpOptions {
        max_distance: 0.03,
        max_iterations: 100,
        ..Default::default()
    };
    let registered = icp(&near, &model, &guess.inverse(), &options).unwrap();
    let pose = registered.transform.inverse();
    let error = pose.translation.vector - truth_pose.translation.vector;
    assert!(error.norm() < 0.003, "{registered:?}");
    assert!(
        pose.rotation.angle_to(&truth_pose.rotation) < 0.02,
        "{registered:?}"
    );
    assert!(
        registered.rmse < 0.002 && registered.fitness > 0.99,
        "{registered:?}"
    );
    // The fitted pose is a guess good enough to start from.
    assert!((tall.shape.pose.translation.vector - truth_pose.translation.vector).norm() < 0.01);
}

#[test]
fn a_cloud_without_a_level_plane_has_no_table() {
    // Only a wall.
    let wall: Vec<[f64; 3]> = (0..2000)
        .map(|i| [0.01 * (i % 50) as f64, 0.0, 0.01 * (i / 50) as f64])
        .collect();
    assert_eq!(
        segment_tabletop(&wall, &TabletopOptions::default()),
        Err(roplat_perception::TabletopError::NoTable)
    );
    // Without `up`, the wall is a plane.
    assert!(fit_plane(&wall, &PlaneOptions::default()).is_some());
}

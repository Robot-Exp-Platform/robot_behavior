//! A can pressed against a box: Euclidean clustering sees one object, and
//! smooth regions grown over the surface normals come apart at the creases,
//! each on one of the two, so the can can still be fitted.

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use parry3d_f64::{query::Ray, shape::SharedShape};
use roplat_perception::{
    RegionOptions, TabletopOptions, estimate_normals, fit_cylinder_on, segment_tabletop,
    smooth_regions,
};

const TABLE_TOP: f64 = 0.7;
/// A 8 x 8 x 12 cm box, and a can of radius 3.5 cm and height 10 cm
/// touching the box's +x face.
const BOX_CENTRE: [f64; 2] = [0.55, 0.0];
const BOX_HALF: [f64; 3] = [0.04, 0.04, 0.06];
const CAN_CENTRE: [f64; 2] = [0.625, 0.0];
const CAN_RADIUS: f64 = 0.035;
const CAN_HEIGHT: f64 = 0.1;
const CAMERA: [f64; 3] = [0.6, -0.55, 1.35];

fn world() -> Vec<(Isometry3<f64>, SharedShape)> {
    vec![
        (
            Isometry3::translation(0.6, 0.0, TABLE_TOP - 0.02),
            SharedShape::cuboid(0.4, 0.5, 0.02),
        ),
        (
            Isometry3::translation(BOX_CENTRE[0], BOX_CENTRE[1], TABLE_TOP + BOX_HALF[2]),
            SharedShape::cuboid(BOX_HALF[0], BOX_HALF[1], BOX_HALF[2]),
        ),
        (
            // parry's cylinders run along y.
            Isometry3::new(
                Vector3::new(CAN_CENTRE[0], CAN_CENTRE[1], TABLE_TOP + CAN_HEIGHT / 2.0),
                Vector3::new(std::f64::consts::FRAC_PI_2, 0.0, 0.0),
            ),
            SharedShape::cylinder(CAN_HEIGHT / 2.0, CAN_RADIUS),
        ),
    ]
}

/// The world seen by a 320 x 240 depth camera at [`CAMERA`] looking at the
/// objects, with about a millimetre of noise, as points in the world.
fn sensed() -> Vec<[f64; 3]> {
    let eye = Vector3::from(CAMERA);
    let z = (Vector3::new(0.6, 0.0, TABLE_TOP) - eye).normalize();
    let x = z.cross(&Vector3::z()).normalize();
    let y = z.cross(&x);
    let rotation = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[x, y, z]));
    let camera = Isometry3::from_parts(
        Translation3::from(eye),
        UnitQuaternion::from_rotation_matrix(&rotation),
    );
    let shapes = world();
    let (f, cx, cy) = (300.0, 159.5, 119.5);
    let mut state = 0x853c_49e6_748f_ea9b_u64;
    let mut noise = move || {
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
    let mut points = Vec::new();
    for v in 0..240 {
        for u in 0..320 {
            let direction = Vector3::new((u as f64 - cx) / f, (v as f64 - cy) / f, 1.0);
            let ray = Ray::new(Point3::from(eye), camera.rotation * direction);
            let hit = shapes
                .iter()
                .filter_map(|(pose, shape)| shape.cast_ray(pose, &ray, 10.0, true))
                .fold(f64::INFINITY, f64::min);
            if hit.is_finite() {
                let p = camera * Point3::from(direction * (hit + noise()));
                points.push([p.x, p.y, p.z]);
            }
        }
    }
    points
}

fn on_box(p: &[f64; 3]) -> bool {
    let margin = 0.004;
    (p[0] - BOX_CENTRE[0]).abs() < BOX_HALF[0] + margin
        && (p[1] - BOX_CENTRE[1]).abs() < BOX_HALF[1] + margin
        && p[2] < TABLE_TOP + 2.0 * BOX_HALF[2] + margin
}

fn on_can(p: &[f64; 3]) -> bool {
    let r = (p[0] - CAN_CENTRE[0]).hypot(p[1] - CAN_CENTRE[1]);
    r < CAN_RADIUS + 0.004 && p[2] < TABLE_TOP + CAN_HEIGHT + 0.004
}

#[test]
fn smooth_regions_separate_a_can_from_the_box_it_touches() {
    let found = segment_tabletop(&sensed(), &TabletopOptions::default()).unwrap();
    // Touching, the two are one Euclidean cluster.
    assert_eq!(found.objects.len(), 1);
    let points = &found.objects[0].points;
    assert!(points.iter().any(on_box) && points.iter().any(on_can));

    // The object's points are 5 mm apart: neighbours within 12 mm, and a
    // can of 3.5 cm turns its normal by about 8° per point.
    let normals = estimate_normals(points, 0.012, CAMERA);
    let options = RegionOptions {
        radius: 0.012,
        smoothness: 12f64.to_radians(),
        min_points: 10,
        ..Default::default()
    };
    let regions = smooth_regions(points, &normals, &options);
    let mut can_side = None;
    for (k, region) in regions.iter().enumerate() {
        // Points within a few millimetres of the contact line lie on both;
        // count only those on one.
        let only = |on: fn(&[f64; 3]) -> bool, other: fn(&[f64; 3]) -> bool| {
            region
                .iter()
                .filter(|&&i| on(&points[i]) && !other(&points[i]))
                .count()
        };
        let (can, r#box) = (only(on_can, on_box), only(on_box, on_can));
        let curved = region
            .iter()
            .filter(|&&i| on_can(&points[i]) && points[i][2] < TABLE_TOP + CAN_HEIGHT - 0.004)
            .count();
        eprintln!(
            "region {k}: {} points, {can} only on the can ({curved} on its side), {box} only on the box",
            region.len(),
            box = r#box
        );
        // Each region lies on one object.
        assert!(
            can.min(r#box) * 20 <= region.len(),
            "region {k} mixes the objects"
        );
        if curved * 10 >= region.len() * 9 && can_side.is_none() {
            can_side = Some(region);
        }
    }
    // The can's side, alone, fits the can.
    let side: Vec<[f64; 3]> = can_side
        .expect("a region on the can's side")
        .iter()
        .map(|&i| points[i])
        .collect();
    let can = fit_cylinder_on(&side, &found.table).expect("a cylinder");
    let centre = can.pose.translation.vector;
    let offset = (centre.x - CAN_CENTRE[0]).hypot(centre.y - CAN_CENTRE[1]);
    eprintln!("can side: {} points, centre {offset:.4} m off", side.len());
    assert!(offset < 0.01, "{offset}");
}

//! Primitive shapes fitted to an object's points: what `roplat_grasp`
//! generates grasps for and `PlanningScene::add_object` places.
//!
//! An object resting on a plane (a table) is fitted in the plane's frame.
//! Its points are projected onto the plane, and the outline of the
//! projection, its convex hull, is fitted by the smallest rectangle around
//! it ([`fit_box_on`]) or by a circle ([`fit_cylinder_on`]); the shape
//! reaches from the plane to the object's top. A depth camera sees an
//! object's top and the sides facing it, which project onto the same
//! outline, so the hidden half does not matter, but the bottom is taken to
//! rest on the plane. [`fit_on`] keeps whichever outline fits better.
//! [`fit_box`] fits a box in any orientation, along the points' principal
//! axes, for an object not resting on anything; [`fit_sphere`] fits a ball
//! anywhere, from the part of its surface that is seen.

use nalgebra as na;
use roplat_description::Geometry;

use crate::{grid::is_finite, segment::Plane};

/// A primitive shape posed in the points' frame.
#[derive(Clone, Debug, PartialEq)]
pub struct FittedShape {
    /// A box (full edge lengths), a cylinder along the pose's `z`, or a
    /// sphere.
    pub geometry: Geometry,
    /// The shape's centre, as URDF places geometry.
    pub pose: na::Isometry3<f64>,
    /// Root-mean-square distance (m) of the fitted outline's points from
    /// the shape's outline; for [`fit_box`] and [`fit_sphere`], of all
    /// points from its surface.
    pub residual: f64,
}

/// Share of an object's heights above the plane below its top. A few
/// noisy returns above the top are ignored.
const TOP_QUANTILE: f64 = 0.99;

/// The upright box resting on `plane` that best encloses the points: the
/// smallest-area rectangle around their outline on the plane, up to their
/// top. `None` for fewer than three finite points or points that do not
/// span an area.
pub fn fit_box_on(points: &[[f64; 3]], plane: &Plane) -> Option<FittedShape> {
    let resting = Resting::new(points, plane)?;
    let rectangle = min_area_rectangle(&resting.outline)?;
    let residual = rms(resting
        .outline
        .iter()
        .map(|p| rectangle.boundary_distance(p)));
    let [a, b] = rectangle.size;
    let height = resting.top;
    let (cos, sin) = (rectangle.angle.cos(), rectangle.angle.sin());
    let rotation = na::Rotation3::from_basis_unchecked(&[
        na::Vector3::new(cos, sin, 0.0),
        na::Vector3::new(-sin, cos, 0.0),
        na::Vector3::z(),
    ]);
    let local = na::Isometry3::from_parts(
        na::Translation3::new(rectangle.centre.x, rectangle.centre.y, height / 2.0),
        na::UnitQuaternion::from_rotation_matrix(&rotation),
    );
    Some(FittedShape {
        geometry: Geometry::Box { size: [a, b, height] },
        pose: resting.frame * local,
        residual,
    })
}

/// The upright cylinder resting on `plane` whose circle best fits the
/// points' outline on the plane, up to their top. `None` for fewer than
/// three finite points or points that do not span an area.
pub fn fit_cylinder_on(points: &[[f64; 3]], plane: &Plane) -> Option<FittedShape> {
    let resting = Resting::new(points, plane)?;
    let (centre, radius) = fit_circle(&resting.outline)?;
    let residual = rms(resting
        .outline
        .iter()
        .map(|p| ((p - centre).norm() - radius).abs()));
    let local = na::Isometry3::translation(centre.x, centre.y, resting.top / 2.0);
    Some(FittedShape {
        geometry: Geometry::Cylinder { radius, length: resting.top },
        pose: resting.frame * local,
        residual,
    })
}

/// The better of [`fit_box_on`] and [`fit_cylinder_on`]: the one whose
/// outline lies closer to the points' outline; the box on a tie.
pub fn fit_on(points: &[[f64; 3]], plane: &Plane) -> Option<FittedShape> {
    let box_fit = fit_box_on(points, plane)?;
    match fit_cylinder_on(points, plane) {
        Some(cylinder) if cylinder.residual < box_fit.residual => Some(cylinder),
        _ => Some(box_fit),
    }
}

/// The sphere that best fits the points, which may cover only the side of
/// it a camera sees: an algebraic least-squares fit (`|p|² = 2 c·p + d`,
/// linear in the centre `c` and `d`) refined by Gauss–Newton steps on the
/// points' distances from the surface. `None` for fewer than four finite
/// points or points that do not span a curved surface (all on one plane or
/// line).
pub fn fit_sphere(points: &[[f64; 3]]) -> Option<FittedShape> {
    let finite: Vec<na::Vector3<f64>> = points
        .iter()
        .filter(|p| is_finite(p))
        .map(|p| na::Vector3::from(*p))
        .collect();
    if finite.len() < 4 {
        return None;
    }
    // Centred for conditioning.
    let mean = finite.iter().sum::<na::Vector3<f64>>() / finite.len() as f64;
    let local: Vec<na::Vector3<f64>> = finite.iter().map(|p| p - mean).collect();
    let mut normal = na::Matrix4::<f64>::zeros();
    let mut rhs = na::Vector4::<f64>::zeros();
    for p in &local {
        let row = na::Vector4::new(2.0 * p.x, 2.0 * p.y, 2.0 * p.z, 1.0);
        normal += row * row.transpose();
        rhs += row * p.norm_squared();
    }
    let x = normal.lu().solve(&rhs)?;
    let mut centre = na::Vector3::new(x[0], x[1], x[2]);
    let squared = x[3] + centre.norm_squared();
    if !(squared > 0.0 && squared.is_finite()) {
        return None;
    }
    let mut radius = squared.sqrt();
    // Gauss–Newton on r_i = |p_i − c| − R over (c, R).
    for _ in 0..20 {
        let mut jtj = na::Matrix4::<f64>::zeros();
        let mut jtr = na::Vector4::<f64>::zeros();
        for p in &local {
            let d = p - centre;
            let distance = d.norm();
            if distance == 0.0 {
                continue;
            }
            let u = d / distance;
            let row = na::Vector4::new(-u.x, -u.y, -u.z, -1.0);
            let residual = distance - radius;
            jtj += row * row.transpose();
            jtr += row * residual;
        }
        let Some(step) = jtj.lu().solve(&(-jtr)) else {
            break;
        };
        centre += step.fixed_rows::<3>(0);
        radius += step[3];
        if step.norm() < 1e-12 {
            break;
        }
    }
    if !(radius > 0.0 && radius.is_finite() && centre.iter().all(|c| c.is_finite())) {
        return None;
    }
    let residual = rms(local.iter().map(|p| ((p - centre).norm() - radius).abs()));
    Some(FittedShape {
        geometry: Geometry::Sphere { radius },
        pose: na::Isometry3::translation(centre.x + mean.x, centre.y + mean.y, centre.z + mean.z),
        residual,
    })
}

/// The box along the points' principal axes that encloses them. `None`
/// for no finite points.
pub fn fit_box(points: &[[f64; 3]]) -> Option<FittedShape> {
    let finite: Vec<na::Point3<f64>> = points
        .iter()
        .filter(|p| is_finite(p))
        .map(|p| na::Point3::from(*p))
        .collect();
    if finite.is_empty() {
        return None;
    }
    let centre = finite.iter().map(|p| p.coords).sum::<na::Vector3<f64>>() / finite.len() as f64;
    let mut covariance = na::Matrix3::zeros();
    for p in &finite {
        let d = p.coords - centre;
        covariance += d * d.transpose();
    }
    let axes = covariance.symmetric_eigen().eigenvectors;
    // A right-handed frame.
    let x = axes.column(0).into_owned();
    let y = axes.column(1).into_owned();
    let rotation = na::Rotation3::from_basis_unchecked(&[x, y, x.cross(&y)]);
    let local: Vec<na::Vector3<f64>> = finite
        .iter()
        .map(|p| rotation.inverse() * (p.coords - centre))
        .collect();
    let lower = local
        .iter()
        .fold(na::Vector3::repeat(f64::INFINITY), |m, v| m.inf(v));
    let upper = local
        .iter()
        .fold(na::Vector3::repeat(f64::NEG_INFINITY), |m, v| m.sup(v));
    let half = (upper - lower) / 2.0;
    let middle = (upper + lower) / 2.0;
    let residual = rms(local.iter().map(|v| {
        let d = (v - middle).abs();
        // Inside: the nearest face; all points are inside.
        (0..3).map(|k| half[k] - d[k]).fold(f64::INFINITY, f64::min)
    }));
    let size = (upper - lower).into();
    Some(FittedShape {
        geometry: Geometry::Box { size },
        pose: na::Isometry3::from_parts(
            na::Translation3::from(centre + rotation * middle),
            na::UnitQuaternion::from_rotation_matrix(&rotation),
        ),
        residual,
    })
}

/// Points resting on a plane, seen in the plane's frame.
struct Resting {
    frame: na::Isometry3<f64>,
    /// Convex hull of the points projected onto the plane, in order.
    outline: Vec<na::Point2<f64>>,
    /// Height of the top above the plane.
    top: f64,
}

impl Resting {
    fn new(points: &[[f64; 3]], plane: &Plane) -> Option<Self> {
        let frame = plane.frame();
        let inverse = frame.inverse();
        let local: Vec<na::Point3<f64>> = points
            .iter()
            .filter(|p| is_finite(p))
            .map(|p| inverse * na::Point3::from(*p))
            .collect();
        if local.len() < 3 {
            return None;
        }
        let mut heights: Vec<f64> = local.iter().map(|p| p.z).collect();
        heights.sort_by(f64::total_cmp);
        let top = heights[((heights.len() - 1) as f64 * TOP_QUANTILE) as usize];
        let outline = convex_hull(local.iter().map(|p| na::Point2::new(p.x, p.y)).collect());
        (outline.len() >= 3 && top > 0.0).then_some(Self { frame, outline, top })
    }
}

/// Counter-clockwise convex hull (Andrew's monotone chain), without
/// collinear points.
pub(crate) fn convex_hull(mut points: Vec<na::Point2<f64>>) -> Vec<na::Point2<f64>> {
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    let cross =
        |o: &na::Point2<f64>, a: &na::Point2<f64>, b: &na::Point2<f64>| (a - o).perp(&(b - o));
    let mut hull: Vec<na::Point2<f64>> = Vec::with_capacity(2 * points.len());
    for pass in [false, true] {
        let start = hull.len();
        let ordered: Box<dyn Iterator<Item = &na::Point2<f64>>> = if pass {
            Box::new(points.iter().rev())
        } else {
            Box::new(points.iter())
        };
        for p in ordered {
            while hull.len() >= start + 2
                && cross(&hull[hull.len() - 2], &hull[hull.len() - 1], p) <= 0.0
            {
                hull.pop();
            }
            hull.push(*p);
        }
        // The last point starts the other chain.
        hull.pop();
    }
    hull
}

/// A rectangle in the plane: centre, side lengths along its own axes, and
/// the angle of its first axis.
struct Rectangle {
    centre: na::Point2<f64>,
    size: [f64; 2],
    angle: f64,
}

impl Rectangle {
    /// Distance of `p` from the rectangle's boundary.
    fn boundary_distance(&self, p: &na::Point2<f64>) -> f64 {
        let rotation = na::Rotation2::new(-self.angle);
        let local = rotation * (p - self.centre);
        let half = na::Vector2::new(self.size[0] / 2.0, self.size[1] / 2.0);
        let outside = (local.abs() - half).sup(&na::Vector2::zeros());
        if outside.norm() > 0.0 {
            outside.norm()
        } else {
            (half - local.abs()).min()
        }
    }
}

/// The smallest-area rectangle around a convex polygon: one of its sides
/// lies along an edge of the polygon (rotating calipers, here by trying
/// every edge).
fn min_area_rectangle(hull: &[na::Point2<f64>]) -> Option<Rectangle> {
    let mut best: Option<(f64, Rectangle)> = None;
    for k in 0..hull.len() {
        let edge = hull[(k + 1) % hull.len()] - hull[k];
        if edge.norm() == 0.0 {
            continue;
        }
        let angle = edge.y.atan2(edge.x);
        let rotation = na::Rotation2::new(-angle);
        let local: Vec<na::Vector2<f64>> = hull.iter().map(|p| rotation * p.coords).collect();
        let lower = local
            .iter()
            .fold(na::Vector2::repeat(f64::INFINITY), |m, v| m.inf(v));
        let upper = local
            .iter()
            .fold(na::Vector2::repeat(f64::NEG_INFINITY), |m, v| m.sup(v));
        let size = upper - lower;
        let area = size.x * size.y;
        if best.as_ref().is_none_or(|(most, _)| area < *most) {
            let centre = na::Point2::from(rotation.inverse() * ((upper + lower) / 2.0));
            best = Some((area, Rectangle { centre, size: [size.x, size.y], angle }));
        }
    }
    best.filter(|(area, _)| *area > 0.0)
        .map(|(_, rectangle)| rectangle)
}

/// Least-squares circle through points (Kåsa's algebraic fit:
/// `x² + y² + D x + E y + F = 0`).
fn fit_circle(points: &[na::Point2<f64>]) -> Option<(na::Point2<f64>, f64)> {
    let rows = points.len();
    let a = na::DMatrix::from_fn(rows, 3, |i, j| match j {
        0 => points[i].x,
        1 => points[i].y,
        _ => 1.0,
    });
    let b = na::DVector::from_fn(rows, |i, _| -(points[i].x.powi(2) + points[i].y.powi(2)));
    let solution = a.svd(true, true).solve(&b, 1e-12).ok()?;
    let centre = na::Point2::new(-solution[0] / 2.0, -solution[1] / 2.0);
    let squared = centre.coords.norm_squared() - solution[2];
    (squared > 0.0).then(|| (centre, squared.sqrt()))
}

fn rms(values: impl Iterator<Item = f64>) -> f64 {
    let (sum, count) = values.fold((0.0, 0usize), |(sum, count), v| (sum + v * v, count + 1));
    (sum / count.max(1) as f64).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_ball_is_fitted_from_the_side_a_camera_sees() {
        // A 4 cm ball at (0.5, 0.1, 0.74), seen from above and one side:
        // only directions within 70° of the view, with ±0.5 mm of noise.
        let (centre, radius) = (na::Vector3::new(0.5, 0.1, 0.74), 0.04);
        let view = na::Vector3::new(0.0, -0.6, 0.8);
        let golden = std::f64::consts::PI * (3.0 - 5f64.sqrt());
        let n = 2000;
        let points: Vec<[f64; 3]> = (0..n)
            .filter_map(|i| {
                let z = 1.0 - 2.0 * (i as f64 + 0.5) / n as f64;
                let r = (1.0 - z * z).sqrt();
                let a = golden * i as f64;
                let direction = na::Vector3::new(r * a.cos(), r * a.sin(), z);
                (direction.dot(&view) > 70f64.to_radians().cos()).then(|| {
                    let noise = 0.0005 * ((i * 7919 % 1000) as f64 / 500.0 - 1.0);
                    let p = centre + direction * (radius + noise);
                    [p.x, p.y, p.z]
                })
            })
            .collect();
        assert!(points.len() > 300 && points.len() < n / 2);
        let fit = fit_sphere(&points).unwrap();
        let Geometry::Sphere { radius: found } = fit.geometry else {
            panic!("{fit:?}")
        };
        assert!((found - radius).abs() < 1e-3, "{found}");
        assert!(
            (fit.pose.translation.vector - centre).norm() < 1e-3,
            "{fit:?}"
        );
        assert!(fit.residual < 5e-4, "{}", fit.residual);
        // Points on a plane span no sphere; too few points none either.
        let flat: Vec<[f64; 3]> = (0..100)
            .map(|i| [(i % 10) as f64 * 0.01, (i / 10) as f64 * 0.01, 0.7])
            .collect();
        assert!(fit_sphere(&flat).is_none());
        assert!(fit_sphere(&points[..3]).is_none());
    }

    fn table() -> Plane {
        Plane::through(&na::Point3::new(0.0, 0.0, 0.7), na::Vector3::z_axis())
    }

    /// The top and four sides of a box of `size` on the table at (x, y),
    /// turned by `yaw`, sampled every 5 mm.
    fn box_points(x: f64, y: f64, yaw: f64, size: [f64; 3]) -> Vec<[f64; 3]> {
        let pose = na::Isometry3::new(
            na::Vector3::new(x, y, 0.7 + size[2] / 2.0),
            na::Vector3::new(0.0, 0.0, yaw),
        );
        let half = size.map(|s| s / 2.0);
        let steps = |extent: f64| ((2.0 * extent / 0.005) as usize).max(1);
        let mut points = Vec::new();
        for i in 0..=steps(half[0]) {
            for j in 0..=steps(half[1]) {
                for k in 0..=steps(half[2]) {
                    let p = na::Vector3::new(
                        -half[0] + 2.0 * half[0] * i as f64 / steps(half[0]) as f64,
                        -half[1] + 2.0 * half[1] * j as f64 / steps(half[1]) as f64,
                        -half[2] + 2.0 * half[2] * k as f64 / steps(half[2]) as f64,
                    );
                    // On the surface, not the bottom.
                    let surface = (0..3).any(|a| (p[a].abs() - half[a]).abs() < 1e-9);
                    if surface && p.z > -half[2] + 1e-9 {
                        points.push((pose * na::Point3::from(p)).coords.into());
                    }
                }
            }
        }
        points
    }

    #[test]
    fn a_box_on_the_table_is_recovered() {
        let points = box_points(0.4, -0.1, 0.5, [0.06, 0.1, 0.15]);
        let fit = fit_box_on(&points, &table()).unwrap();
        let Geometry::Box { size } = fit.geometry else {
            panic!("{fit:?}")
        };
        let mut sides = [size[0], size[1]];
        sides.sort_by(f64::total_cmp);
        assert!(
            (sides[0] - 0.06).abs() < 1e-6 && (sides[1] - 0.1).abs() < 1e-6,
            "{size:?}"
        );
        assert!((size[2] - 0.15).abs() < 1e-6);
        let centre = fit.pose.translation.vector;
        assert!(
            (centre - na::Vector3::new(0.4, -0.1, 0.775)).norm() < 1e-6,
            "{centre}"
        );
        // The box's z is up and its sides follow the yaw, modulo a quarter turn.
        let z = fit.pose.rotation * na::Vector3::z();
        assert!((z - na::Vector3::z()).norm() < 1e-9);
        let x = fit.pose.rotation * na::Vector3::x();
        let yaw = x.y.atan2(x.x).rem_euclid(std::f64::consts::FRAC_PI_2);
        assert!((yaw - 0.5).abs() < 1e-6, "{yaw}");
        assert!(fit.residual < 1e-9);
        assert_eq!(fit_on(&points, &table()).unwrap().geometry, fit.geometry);
    }

    #[test]
    fn a_cylinder_on_the_table_is_recovered() {
        let mut points = Vec::new();
        for k in 0..72 {
            let angle = std::f64::consts::TAU * k as f64 / 72.0;
            for h in 0..=24 {
                points.push([
                    0.3 + 0.04 * angle.cos(),
                    0.2 + 0.04 * angle.sin(),
                    0.7 + 0.005 * h as f64,
                ]);
            }
        }
        let fit = fit_on(&points, &table()).unwrap();
        let Geometry::Cylinder { radius, length } = fit.geometry else {
            panic!("{fit:?}")
        };
        assert!(
            (radius - 0.04).abs() < 1e-6 && (length - 0.12).abs() < 1e-6,
            "{fit:?}"
        );
        let centre = fit.pose.translation.vector;
        assert!(
            (centre - na::Vector3::new(0.3, 0.2, 0.76)).norm() < 1e-6,
            "{centre}"
        );
        // A box around the same points fits its outline worse.
        assert!(fit_box_on(&points, &table()).unwrap().residual > 10.0 * fit.residual);
    }

    #[test]
    fn a_box_anywhere_follows_its_principal_axes() {
        let points = box_points(0.0, 0.0, 0.3, [0.05, 0.1, 0.3]);
        let fit = fit_box(&points).unwrap();
        let Geometry::Box { size } = fit.geometry else {
            panic!("{fit:?}")
        };
        let mut sorted = size;
        sorted.sort_by(f64::total_cmp);
        assert!(
            (sorted[0] - 0.05).abs() < 1e-6 && (sorted[1] - 0.1).abs() < 1e-6,
            "{size:?}"
        );
        // The lowest points sampled are 5 mm above the open bottom.
        assert!((sorted[2] - 0.295).abs() < 1e-6, "{size:?}");
    }

    #[test]
    fn degenerate_input_gives_none() {
        assert!(fit_box_on(&[[0.0, 0.0, 0.8]; 3], &table()).is_none());
        assert!(
            fit_cylinder_on(
                &[[0.0, 0.0, 0.8], [0.1, 0.0, 0.8], [0.2, 0.0, 0.8]],
                &table()
            )
            .is_none()
        );
        assert!(fit_box(&[]).is_none());
    }
}

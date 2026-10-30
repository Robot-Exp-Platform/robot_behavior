//! Joint-space planning around obstacles given only by a validity check.

use roplat_trajectory::{
    PlanError, PlannerOptions, plan_joint_motion, plan_joint_path, segment_is_valid,
    simplify_motion, smooth_motion,
};

/// A wall at x ∈ [0.4, 0.6] across the unit square, with a gap at y ∈ [0.8, 0.9].
fn outside_wall(q: &[f64; 2]) -> bool {
    !((0.4..=0.6).contains(&q[0]) && !(0.8..=0.9).contains(&q[1]))
}

fn check(path: &[[f64; 2]], start: [f64; 2], goal: [f64; 2], valid: impl Fn(&[f64; 2]) -> bool) {
    assert_eq!(path.first(), Some(&start));
    assert_eq!(path.last(), Some(&goal));
    for pair in path.windows(2) {
        assert!(
            segment_is_valid(&pair[0], &pair[1], 0.005, &valid),
            "{pair:?}"
        );
    }
}

#[test]
fn finds_the_gap_and_repeats_for_a_seed() {
    let (start, goal) = ([0.1, 0.1], [0.9, 0.1]);
    let options = PlannerOptions { max_step: 0.1, resolution: 0.005, ..Default::default() };
    let path = plan_joint_path(start, goal, [0.0; 2], [1.0; 2], outside_wall, options).unwrap();
    check(&path, start, goal, outside_wall);
    assert!(path.iter().any(|q| q[1] > 0.75), "{path:?}");
    let again = plan_joint_path(start, goal, [0.0; 2], [1.0; 2], outside_wall, options).unwrap();
    assert_eq!(path, again);
}

#[test]
fn a_clear_line_is_one_segment() {
    let options = PlannerOptions::default();
    let path = plan_joint_path(
        [0.1, 0.1],
        [0.3, 0.9],
        [0.0; 2],
        [1.0; 2],
        outside_wall,
        options,
    );
    assert_eq!(path.unwrap(), [[0.1, 0.1], [0.3, 0.9]]);
}

#[test]
fn reports_invalid_ends_and_closed_walls() {
    let options = PlannerOptions { max_iterations: 300, ..Default::default() };
    let (lower, upper) = ([0.0; 2], [1.0; 2]);
    assert_eq!(
        plan_joint_path([0.5, 0.1], [0.9, 0.1], lower, upper, outside_wall, options),
        Err(PlanError::InvalidStart)
    );
    assert_eq!(
        plan_joint_path([0.1, 0.1], [1.5, 0.1], lower, upper, outside_wall, options),
        Err(PlanError::InvalidGoal)
    );
    let closed = |q: &[f64; 2]| !(0.4..=0.6).contains(&q[0]);
    assert_eq!(
        plan_joint_path([0.1, 0.1], [0.9, 0.1], lower, upper, closed, options),
        Err(PlanError::NotFound { iterations: 300 })
    );
}

/// A planar 2R arm (links 0.5 m) whose elbow, forearm midpoint and tip must
/// stay out of a disc in front of it; the tip crosses from one side to the other.
#[test]
fn plans_an_arm_around_a_disc() {
    let points = |q: &[f64; 2]| {
        let elbow = [0.5 * q[0].cos(), 0.5 * q[0].sin()];
        let tip = [
            elbow[0] + 0.5 * (q[0] + q[1]).cos(),
            elbow[1] + 0.5 * (q[0] + q[1]).sin(),
        ];
        [
            elbow,
            [(elbow[0] + tip[0]) / 2.0, (elbow[1] + tip[1]) / 2.0],
            tip,
        ]
    };
    let clear = |q: &[f64; 2]| points(q).iter().all(|p| (p[0] - 0.8).hypot(p[1]) > 0.15);
    let (start, goal) = ([0.6, -0.4], [-0.6, 0.4]);
    let (lower, upper) = ([-std::f64::consts::PI; 2], [std::f64::consts::PI; 2]);
    assert!(
        !segment_is_valid(&start, &goal, 0.005, clear),
        "the straight move must collide"
    );
    let path =
        plan_joint_path(start, goal, lower, upper, clear, PlannerOptions::default()).unwrap();
    check(&path, start, goal, clear);
}

/// A wall 1 cm thick at x ∈ [0.495, 0.505], open above y = 0.8.
fn crosses_thin_wall(a: &[f64; 2], b: &[f64; 2]) -> bool {
    let (x0, x1) = (0.495, 0.505);
    // The part of the segment with x in the wall, as fractions of it.
    let (lo, hi) = if a[0] == b[0] {
        if (x0..=x1).contains(&a[0]) {
            (0.0, 1.0)
        } else {
            return false;
        }
    } else {
        let (t0, t1) = ((x0 - a[0]) / (b[0] - a[0]), (x1 - a[0]) / (b[0] - a[0]));
        (t0.min(t1).max(0.0), t0.max(t1).min(1.0))
    };
    // y is linear in t, so its lowest value on that part is at an end.
    lo <= hi && [lo, hi].iter().any(|t| a[1] + t * (b[1] - a[1]) < 0.8)
}

fn motion_clear(a: &[f64; 2], b: &[f64; 2]) -> bool {
    !crosses_thin_wall(a, b)
}

#[test]
fn a_motion_check_does_not_step_over_a_thin_wall() {
    let (start, goal) = ([0.2, 0.2], [0.75, 0.2]);
    let (lower, upper) = ([0.0; 2], [1.0; 2]);
    let point_free = |q: &[f64; 2]| motion_clear(q, q);
    // Samples 0.4 apart land on both sides of the wall: one segment, through it.
    let coarse = PlannerOptions { resolution: 0.4, ..Default::default() };
    let sampled = plan_joint_path(start, goal, lower, upper, point_free, coarse).unwrap();
    assert_eq!(sampled, [start, goal]);
    assert!(crosses_thin_wall(&start, &goal));

    let path = plan_joint_motion(start, goal, lower, upper, motion_clear, coarse).unwrap();
    assert_eq!((path[0], *path.last().unwrap()), (start, goal));
    for pair in path.windows(2) {
        assert!(motion_clear(&pair[0], &pair[1]), "{pair:?}");
    }
    assert!(path.iter().any(|q| q[1] > 0.8), "{path:?}");
    let again = plan_joint_motion(start, goal, lower, upper, motion_clear, coarse).unwrap();
    assert_eq!(path, again);

    // An end in the wall is refused as invalid.
    assert_eq!(
        plan_joint_motion([0.5, 0.1], goal, lower, upper, motion_clear, coarse),
        Err(PlanError::InvalidStart)
    );
}

#[test]
fn simplifying_and_smoothing_by_motion_keep_segments_off_the_wall() {
    let path = [[0.2, 0.2], [0.3, 0.9], [0.7, 0.9], [0.75, 0.2]];
    let simple = simplify_motion(&path, motion_clear);
    // The wall blocks the direct line, so a waypoint above it stays.
    assert!(simple.len() >= 3, "{simple:?}");
    for pair in simple.windows(2) {
        assert!(motion_clear(&pair[0], &pair[1]), "{pair:?}");
    }
    let smooth = smooth_motion(&simple, 3, motion_clear);
    assert_eq!((smooth[0], *smooth.last().unwrap()), (path[0], path[3]));
    for pair in smooth.windows(2) {
        assert!(motion_clear(&pair[0], &pair[1]), "{pair:?}");
    }
}

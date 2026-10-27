//! Joint-space planning around obstacles given only by a validity check.

use roplat_trajectory::{PlanError, PlannerOptions, plan_joint_path, segment_is_valid};

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

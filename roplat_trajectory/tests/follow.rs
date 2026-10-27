//! The follower against a manual clock: what it commands, when it is done and
//! when it refuses.

use std::{
    sync::{
        Arc,
        atomic::{AtomicI64, Ordering},
    },
    time::Duration,
};

use roplat_behavior::{
    JointSample, JointState, StateView,
    msgs::{Clock, JointTrajectoryPoint, ManualClock, Stamp},
};
use roplat_trajectory::{JointTrajectoryFollower, TrackingError};

const MS: i64 = 1_000_000;

fn point(ms: i64, q: [f64; 2]) -> JointTrajectoryPoint<2> {
    JointTrajectoryPoint { time_from_start: ms * MS, q, dq: [0.; 2], ddq: [0.; 2] }
}

fn at(q: [f64; 2]) -> JointState<2> {
    StateView::from_meas(JointSample::EMPTY.with_q(q))
}

fn ms(ms: u64) -> Duration {
    Duration::from_millis(ms)
}

/// From rest at the origin to rest at `[1, -2]` in 100 ms.
fn rest_to_rest() -> Vec<JointTrajectoryPoint<2>> {
    vec![point(0, [0., 0.]), point(100, [1., -2.])]
}

fn follower(clock: &Arc<ManualClock>) -> JointTrajectoryFollower<Arc<ManualClock>, 2> {
    JointTrajectoryFollower::new(rest_to_rest(), clock.clone()).unwrap()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

#[test]
fn a_segment_passes_through_its_points_and_blends_between() {
    let follower = follower(&Arc::new(ManualClock::simulation()));
    assert_eq!(follower.duration(), ms(100));
    assert_eq!(follower.sample(ms(0)).q, [0., 0.]);
    assert_eq!(follower.sample(ms(100)).q, [1., -2.]);
    assert_eq!(follower.sample(ms(500)).q, [1., -2.]);

    // Rest to rest is symmetric: halfway in time is halfway in position, at
    // the peak speed 15/8 of the mean.
    let middle = follower.sample(ms(50));
    assert!(close(middle.q[0], 0.5) && close(middle.q[1], -1.));
    assert!(close(middle.dq[0], 15. / 8. * 10.) && close(middle.ddq[0], 0.));
    let start = follower.sample(Duration::from_nanos(1));
    assert!(start.dq[0].abs() < 1e-6 && start.ddq[0].abs() < 1e-3);
}

#[test]
fn velocity_and_acceleration_carry_through_an_interior_point() {
    let mut via = point(100, [0.5, 0.5]);
    via.dq = [2., -1.];
    via.ddq = [3., 0.];
    let points = vec![point(0, [0., 0.]), via, point(200, [1., 0.])];
    let follower = JointTrajectoryFollower::new(points, ManualClock::simulation()).unwrap();
    for t in [
        ms(100) - Duration::from_nanos(1),
        ms(100),
        ms(100) + Duration::from_nanos(1),
    ] {
        let sample = follower.sample(t);
        assert!((sample.q[0] - 0.5).abs() < 1e-6, "{sample:?}");
        assert!((sample.dq[0] - 2.).abs() < 1e-4 && (sample.dq[1] + 1.).abs() < 1e-4);
        assert!((sample.ddq[0] - 3.).abs() < 1e-2, "{sample:?}");
    }
}

#[test]
fn trajectories_that_cannot_be_followed_are_rejected() {
    let reject = |points| JointTrajectoryFollower::new(points, ManualClock::simulation()).err();
    assert_eq!(reject(vec![]), Some(TrackingError::Empty));
    assert_eq!(
        reject(vec![point(-1, [0.; 2])]),
        Some(TrackingError::Timing { index: 0 })
    );
    assert_eq!(
        reject(vec![point(0, [0.; 2]), point(0, [1.; 2])]),
        Some(TrackingError::Timing { index: 1 })
    );
    let mut bad = point(10, [0.; 2]);
    bad.ddq[1] = f64::NAN;
    assert_eq!(
        reject(vec![point(0, [0.; 2]), bad]),
        Some(TrackingError::NonFinite { index: 1 })
    );
}

#[test]
fn commands_follow_the_clock_not_the_number_of_cycles() {
    let clock = Arc::new(ManualClock::simulation());
    let mut follower = follower(&clock);
    let anywhere = at([7., 7.]);

    assert_eq!(follower.step(&anywhere), Ok(([0., 0.], false)));
    clock.advance(ms(30));
    let expected = follower.sample(ms(30)).q;
    assert_eq!(follower.step(&anywhere), Ok((expected, false)));
    // A second cycle at the same instant commands the same point.
    assert_eq!(follower.step(&anywhere), Ok((expected, false)));
    // A late cycle commands where the trajectory is by then.
    clock.advance(ms(55));
    assert_eq!(
        follower.step(&anywhere),
        Ok((follower.sample(ms(85)).q, false))
    );
    clock.advance(ms(15));
    assert_eq!(follower.step(&anywhere), Ok(([1., -2.], true)));
}

#[test]
fn a_finished_run_starts_over_on_the_next_step() {
    let clock = Arc::new(ManualClock::simulation());
    let mut follower = follower(&clock);
    follower.step(&at([0.; 2])).unwrap();
    clock.advance(ms(100));
    assert_eq!(follower.step(&at([0.; 2])), Ok(([1., -2.], true)));
    clock.advance(ms(40));
    assert_eq!(follower.step(&at([1., -2.])), Ok(([0., 0.], false)));
}

#[test]
fn done_waits_until_the_goal_is_within_tolerance() {
    let clock = Arc::new(ManualClock::simulation());
    let mut follower = follower(&clock).with_goal_tolerance([0.01; 2], ms(50));
    follower.step(&at([0.; 2])).unwrap();
    clock.advance(ms(100));
    assert_eq!(follower.step(&at([0.9, -2.])), Ok(([1., -2.], false)));
    clock.advance(ms(20));
    assert_eq!(follower.step(&at([0.995, -2.005])), Ok(([1., -2.], true)));
}

#[test]
fn a_goal_not_reached_in_time_is_refused() {
    let clock = Arc::new(ManualClock::simulation());
    let mut follower = follower(&clock).with_goal_tolerance([0.01; 2], ms(50));
    follower.step(&at([0.; 2])).unwrap();
    clock.advance(ms(150));
    assert_eq!(follower.step(&at([1., -1.9])), Ok(([1., -2.], false)));
    clock.advance(ms(1));
    let Err(TrackingError::GoalTolerance { joint, error, tolerance, goal_time }) =
        follower.step(&at([1., -1.9]))
    else {
        panic!("the late goal should be refused")
    };
    assert_eq!((joint, tolerance, goal_time), (1, 0.01, ms(50)));
    assert!(close(error, 0.1));
}

#[test]
fn straying_from_the_path_is_refused() {
    let clock = Arc::new(ManualClock::simulation());
    let mut follower = follower(&clock).with_path_tolerance([0.05, f64::INFINITY]);
    follower.step(&at([0.; 2])).unwrap();
    clock.advance(ms(50));
    // Joint 1 is unchecked, however far it is.
    assert!(follower.step(&at([0.5, 5.])).is_ok());
    clock.advance(ms(10));
    let Err(TrackingError::PathTolerance { joint, error, tolerance, at: when }) =
        follower.step(&at([0.5, -1.]))
    else {
        panic!("the lagging joint should be refused")
    };
    // s = 0.6 on the rest-to-rest blend 10s³ - 15s⁴ + 6s⁵.
    assert_eq!((joint, tolerance, when), (0, 0.05, ms(60)));
    assert!(close(error, 0.68256 - 0.5));
    // The refusal ended the run.
    assert_eq!(follower.step(&at([0., -1.])), Ok(([0., 0.], false)));
}

#[test]
fn a_checked_tolerance_needs_a_measured_position() {
    let clock = Arc::new(ManualClock::simulation());
    let mut unchecked = follower(&clock);
    assert!(unchecked.step(&StateView::default()).is_ok());
    let mut checked = follower(&clock).with_path_tolerance([0.1; 2]);
    assert_eq!(
        checked.step(&StateView::default()),
        Err(TrackingError::Unobserved)
    );
    let mut nan = follower(&clock).with_path_tolerance([0.1; 2]);
    assert!(matches!(
        nan.step(&at([f64::NAN, 0.])),
        Err(TrackingError::PathTolerance { joint: 0, .. })
    ));
}

/// A clock that can be set back, which `ManualClock` refuses.
struct Rewindable(AtomicI64);

impl Clock for Rewindable {
    fn id(&self) -> u32 {
        9
    }
    fn now(&self) -> Stamp {
        Stamp::new(9, self.0.load(Ordering::SeqCst))
    }
    fn sleep_until(&self, _: Stamp) {
        unreachable!("the follower never waits")
    }
}

#[test]
fn a_clock_that_goes_back_is_refused() {
    let clock = Arc::new(Rewindable(AtomicI64::new(10 * MS)));
    let mut follower = JointTrajectoryFollower::new(rest_to_rest(), clock.clone()).unwrap();
    follower.step(&at([0.; 2])).unwrap();
    clock.0.store(4 * MS, Ordering::SeqCst);
    assert_eq!(
        follower.step(&at([0.; 2])),
        Err(TrackingError::ClockWentBack { nanos: 6 * MS })
    );
}

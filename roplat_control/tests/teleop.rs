//! Operator input to velocity commands, and the controller's safety behavior.

use std::{cell::Cell, rc::Rc, time::Duration};

use roplat_behavior::msgs::InputAxes;
use roplat_control::{TeleopMapping, axes_to_velocity, cartesian_teleop_control};

fn axes(axes: [f64; 6], buttons: u32) -> InputAxes {
    InputAxes { axes, buttons, reserved: 0 }
}

fn close(a: [f64; 6], b: [f64; 6]) -> bool {
    a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12)
}

#[test]
fn shapes_scales_and_guards_samples() {
    let mapping = TeleopMapping {
        deadzone: 0.2,
        linear_speed: 0.1,
        angular_speed: 1.0,
        ..Default::default()
    };
    // Inside the deadzone nothing moves; past it the command starts from zero.
    let v = axes_to_velocity(&axes([0.15, -0.2, 0.6, 1.0, -1.0, 0.0], 0), &mapping);
    assert!(close(v, [0.0, 0.0, 0.05, 1.0, -1.0, 0.0]), "{v:?}");
    // Out-of-range and non-finite axes.
    let v = axes_to_velocity(
        &axes([3.0, f64::NAN, f64::INFINITY, -7.0, 0.0, 0.0], 0),
        &mapping,
    );
    assert!(close(v, [0.1, 0.0, 0.0, -1.0, 0.0, 0.0]), "{v:?}");
    // Mounting signs.
    let flipped = TeleopMapping { signs: [-1.0, 1.0, 1.0, 1.0, 1.0, -1.0], ..mapping };
    let v = axes_to_velocity(&axes([1.0, 0.0, 0.0, 0.0, 0.0, 1.0], 0), &flipped);
    assert!(close(v, [-0.1, 0.0, 0.0, 0.0, 0.0, -1.0]), "{v:?}");
}

#[test]
fn the_dead_man_button_gates_motion() {
    let mapping = TeleopMapping { deadzone: 0.0, enable_button: Some(3), ..Default::default() };
    let pushed = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    assert_eq!(axes_to_velocity(&axes(pushed, 0b0111), &mapping), [0.0; 6]);
    assert_eq!(axes_to_velocity(&axes(pushed, 0b1000), &mapping)[0], 0.1);
    let bad_button = TeleopMapping { enable_button: Some(40), ..mapping };
    assert_eq!(
        axes_to_velocity(&axes(pushed, u32::MAX), &bad_button),
        [0.0; 6]
    );
}

#[test]
fn the_controller_ramps_times_out_and_stops() {
    let mapping = TeleopMapping {
        deadzone: 0.0,
        linear_speed: 0.1,
        input_timeout: Duration::from_millis(30),
        max_linear_accel: 1.0,
        ..Default::default()
    };
    let sample = Rc::new(Cell::new(None));
    let stopping = Rc::new(Cell::new(false));
    let (s, st) = (sample.clone(), stopping.clone());
    let mut control =
        cartesian_teleop_control::<(), _, _>(move || s.take(), mapping, move || st.get());
    let dt = Duration::from_millis(10);

    // No input yet: nothing moves.
    assert_eq!(control((), dt), ([0.0; 6], false));
    // Full deflection ramps at 1 m/s²: 0.01 m/s per 10 ms cycle up to 0.1 m/s.
    sample.set(Some(axes([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0)));
    let mut vx = vec![];
    for _ in 0..12 {
        vx.push(control((), dt).0[0]);
        if vx.len() < 3 {
            sample.set(Some(axes([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0)));
        }
    }
    assert!(
        (vx[0] - 0.01).abs() < 1e-12 && (vx[1] - 0.02).abs() < 1e-12,
        "{vx:?}"
    );
    // The last sample arrived at cycle 3; after 30 ms more the command decays.
    assert!(
        vx[4] > vx[3] - 1e-12,
        "still following before the timeout: {vx:?}"
    );
    assert!(vx[11] < vx[6], "decays after the timeout: {vx:?}");

    // Fresh input then a stop request: ramps to zero, then reports done.
    sample.set(Some(axes([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0)));
    control((), dt);
    stopping.set(true);
    let mut done = false;
    for _ in 0..20 {
        let (command, d) = control((), dt);
        if d {
            assert_eq!(command, [0.0; 6]);
            done = true;
            break;
        }
    }
    assert!(done);
}

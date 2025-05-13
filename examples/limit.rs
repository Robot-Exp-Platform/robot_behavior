use robot_behavior::ArmParam;
use std::f64::consts::FRAC_PI_2;

struct ExRobot {}

impl ArmParam<7> for ExRobot {
    const DH: [[f64; 4]; 7] = [
        [0., 0.333, 0., 0.],
        [0., 0., 0., -FRAC_PI_2],
        [0., 0.316, 0., FRAC_PI_2],
        [0., 0., 0.0825, FRAC_PI_2],
        [0., 0.384, -0.0825, -FRAC_PI_2],
        [0., 0., 0., FRAC_PI_2],
        [0., 0., 0.088, FRAC_PI_2],
    ];
    const JOINT_MIN: [f64; 7] = [
        -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
    ];
    const JOINT_MAX: [f64; 7] = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
}

fn main() {
    let q = [1.; 7];
    let pose = ExRobot::forward_kinematics(&q);
    println!("pose: {:?}", pose.axis_angle());
    println!("pose: {:?}", pose.euler());
    println!("pose: {:?}", pose.homo());
    println!("pose: {:?}", pose.position());
    println!("pose: {:?}", pose.quat());
}

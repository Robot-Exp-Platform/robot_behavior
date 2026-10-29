#[allow(unused_imports)]
pub mod types {
    use roplat_behavior::Pose;

    #[cxx::bridge]
    mod to_cxx_bridge {
        extern "Rust" {
            type CxxPose;
            #[Self = "CxxPose"]
            fn Euler(tran: [f64; 3], rot: [f64; 3]) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn Quat(tran: [f64; 3], rot: [f64; 4]) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn Homo(pose: [f64; 16]) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn AxisAngle(tran: [f64; 3], axis: [f64; 3], angle: f64) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn Position(tran: [f64; 3]) -> Box<CxxPose>;
            fn to_vec(&self) -> Vec<f64>;
            #[Self = "CxxPose"]
            fn from_vec(vec: Vec<f64>) -> Box<CxxPose>;
        }
    }

    pub use to_cxx_bridge::*;

    struct CxxPose(Pose);

    impl CxxPose {
        #[allow(non_snake_case)]
        fn Euler(tran: [f64; 3], rot: [f64; 3]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_euler(tran, rot)))
        }
        #[allow(non_snake_case)]
        fn Quat(tran: [f64; 3], rot: [f64; 4]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_quat_array(tran, rot)))
        }
        #[allow(non_snake_case)]
        fn Homo(pose: [f64; 16]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_homo(pose)))
        }
        #[allow(non_snake_case)]
        fn AxisAngle(tran: [f64; 3], axis: [f64; 3], angle: f64) -> Box<Self> {
            Box::new(CxxPose(Pose::from_axis_angle(tran, axis, angle)))
        }
        #[allow(non_snake_case)]
        fn Position(tran: [f64; 3]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_position(tran)))
        }

        fn to_vec(&self) -> Vec<f64> {
            self.0.into()
        }
        fn from_vec(vec: Vec<f64>) -> Box<Self> {
            Box::new(CxxPose(vec.into()))
        }
    }
}

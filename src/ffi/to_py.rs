use pyo3::{PyResult, pyclass, pymethods};

use crate::ArmState;

macro_rules! py_arm_state {
    ($name: ident, $dof: expr) => {
        #[pyclass]
        pub struct $name(ArmState<$dof>);

        #[pymethods]
        impl $name {
            fn echo(&self) -> PyResult<String> {
                Ok(format!("{}", self.0))
            }

            #[getter]
            fn joint(&self) -> PyResult<Option<[f64; $dof]>> {
                Ok(self.0.joint)
            }

            #[getter]
            fn joint_vel(&self) -> PyResult<Option<[f64; $dof]>> {
                Ok(self.0.joint_vel)
            }

            #[getter]
            fn joint_acc(&self) -> PyResult<Option<[f64; $dof]>> {
                Ok(self.0.joint_acc)
            }
        }

        impl From<ArmState<$dof>> for $name {
            fn from(state: ArmState<$dof>) -> Self {
                $name(state)
            }
        }
    };
}

py_arm_state!(ArmState1, 1);
py_arm_state!(ArmState2, 2);
py_arm_state!(ArmState3, 3);
py_arm_state!(ArmState4, 4);
py_arm_state!(ArmState5, 5);
py_arm_state!(ArmState6, 6);
py_arm_state!(ArmState7, 7);

#[macro_export]
macro_rules! py_robot_behavior {
    ($pyname: ident($name: ident)) => {};
}

#[macro_export]
macro_rules! py_arm_behavior {
    ($pyname: ident<$dof: expr>($name: ident)$(,$macros:ident)*) => {
        py_arm_behavior!($pyname<$dof>($name)$(,$macros)*,{});
    };
    ($pyname: ident<$dof: expr>($name: ident)$(,$macros:ident)*,{$block:block}) => {
        #[pyclass(name = stringify!($name))]
        pub struct $pyname($name);

        #[pymethods]
        impl $pyname {
            $block

            py_robot_behavior!($pyname($name));
            $($macros)*

            fn set_load(&self, m: f64, x: [f64; 3], i: [f64; 9]) -> PyResult<()> {
                self.0.set_load(m, x, i)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_preplanned_motion {}

#[macro_export]
macro_rules! py_arm_preplanned_motion_ext {
    ($pyname: ident<$dof: expr>($name: ident)) => {
        fn move_joint(target: &[f64; N], speed: f64) -> PyResult<()> {
            self.0.move_joint(target, speed)
        }
        fn move_joint_async(target: &[f64; N], speed: f64) -> PyResult<()> {
            self.0.move_joint_async(target, speed)
        }
        fn move_joint_rel(target: &[f64; N], speed: f64) -> PyResult<()> {
            self.0.move_joint_rel(target, speed)
        }
        fn move_joint_rel_async(target: &[f64; N], speed: f64) -> PyResult<()> {
            self.0.move_joint_rel_async(target, speed)
        }
        // fn move_joint_with_quat(
        //     target: &na::Isometry3<f64>,
        //     quat: &na::Quaternion<f64>,
        //     speed: f64,
        // ) -> PyResult<()> {
        //     self.0.move_joint_with_quat(target, quat, speed)
        // }
        // fn move_joint_with_quat_async(
        //     target: &na::Isometry3<f64>,
        //     quat: &na::Quaternion<f64>,
        //     speed: f64,
        // ) -> PyResult<()> {
        //     self.0.move_joint_with_quat_async(target, quat, speed)
        // }
        fn move_linear_with_euler(target: &[f64; 6], speed: f64) -> PyResult<()> {
            self.0.move_linear_with_euler(target, speed)
        }
        fn move_linear_with_euler_async(target: &[f64; 6], speed: f64) -> PyResult<()> {
            self.0.move_linear_with_euler_async(target, speed)
        }
        fn move_linear_with_homo(target: &[f64; 6], speed: f64) -> PyResult<()> {
            self.0.move_linear_with_homo(target, speed)
        }
        fn move_linear_with_homo_async(target: &[f64; 6], speed: f64) -> PyResult<()> {
            self.0.move_linear_with_homo_async(target, speed)
        }
        // fn move_path_prepare
        // fn move_path_start
        fn move_path_prepare_from_file(&self, file_path: &str) -> PyResult<()> {
            self.0.load_path_from_file(file_path)
        }
        fn move_path_from_file(&self, file_path: &str) -> PyResult<()> {
            self.0.move_path_from_file(file_path)
        }
    };
}

#[macro_export]
macro_rules! py_arm_streaming_handle {}

#[macro_export]
macro_rules! py_arm_streaming_motion {}

#[macro_export]
macro_rules! py_arm_streaming_motion_ext {}

#[macro_export]
macro_rules! py_arm_real_time_control {}

#[macro_export]
macro_rules! py_arm_real_time_control_ext {}

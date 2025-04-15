use pyo3::{PyResult, pyclass, pymethods};

use crate::ArmState;

#[macro_export]
macro_rules! py_mathods {
    ($(fn $name: ident(&self $(, $arg: ident: $arg_type: ty)*) -> $ret: ty;)*) => {
        $(
            fn $name(&self $(, $arg: $arg_type)*) -> $ret {
                let ret = self.0.$name($($arg),*);
                Ok(ret)
            }
        )*
    };
    ($(fn $name: ident(&self $(, $arg: ident: $arg_type: ty)*) -> RobotResult<$ret: ty>;)*) => {
        $(
            fn $name(&self $(, $arg: $arg_type)*) -> RobotResult<$ret> {
                let ret = self.0.$name($($arg),*);
                Ok(ret)
            }
        )*
    };
    ($(fn $name: ident(&mut self $(, $arg: ident: $arg_type: ty)*) -> $ret: ty;)*) => {
        $(
            fn $name(&mut self $(, $arg: $arg_type)*) -> $ret {
                let ret = self.0.$name($($arg),*);
                Ok(ret)
            }
        )*
    };
    ($(fn $name: ident(&mut self $(, $arg: ident: $arg_type: ty)*) -> RobotResult<$ret: ty>;)*) => {
        $(
            fn $name(&mut self $(, $arg: $arg_type)*) -> RobotResult<$ret> {
                let ret = self.0.$name($($arg),*);
                Ok(ret)
            }
        )*
    };
}

#[macro_export]
macro_rules! py_robot_behavior {
    ($pyname: ident($name: ident)) => {
        py_mathods! {
            fn version(&self) -> String;
        }
        py_mathods! {
            fn init(&mut self) -> RobotResult<()>;
            fn shutdown(&mut self) -> RobotResult<()>;
            fn enable(&mut self) -> RobotResult<()>;
            fn disable(&mut self) -> RobotResult<()>;
            fn reset(&mut self) -> RobotResult<()>;
            fn is_moving(&mut self) -> bool;
            fn stop(&mut self) -> RobotResult<()>;
            fn pause(&mut self) -> RobotResult<()>;
            fn resume(&mut self) -> RobotResult<()>;
            fn emergency_stop(&mut self) -> RobotResult<()>;
            fn clear_emergency_stop(&mut self) -> RobotResult<()>;
        }
    };
}

#[macro_export]
macro_rules! py_arm_behavior {
    ($pyname: ident<{ $dof: expr }>($name: ident)$(,$macros:ident)*) => {
        py_arm_behavior!($pyname<{ $dof }>($name)$(,$macros)*, {});
    };
    ($pyname: ident<{ $dof: expr }>($name: ident)$(,$macros:ident)*, $block:block) => {
        #[pyclass(name = $name)]
        pub struct $pyname($name);

        #[pymethods]
        impl $pyname {

            py_robot_behavior!($pyname($name));
            $($macros)*

            py_mathods! {
                fn set_load(&self, m: f64, x: [f64; 3], i: [f64; 9]) -> PyResult<()>;
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_preplanned_motion {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {};
}

#[macro_export]
macro_rules! py_arm_preplanned_motion_ext {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {
        py_mathods! {
            fn move_joint(&self, target: &[f64; $dof], speed: f64) -> RobotResult<()>;
            fn move_joint_async(&self, target: &[f64; $dof], speed: f64) -> RobotResult<()>;
            fn move_joint_rel(&self, target: &[f64; $dof], speed: f64) -> RobotResult<()>;
            fn move_joint_rel_async(&self, target: &[f64; $dof], speed: f64) -> RobotResult<()>;
        }
        // py_mathods! {
        //     fn move_joint_with_quat(&self, target: &na::Isometry3<f64>,  speed: f64) -> RobotResult<()>;
        //     fn move_joint_with_quat_async(&self, target: &na::Isometry3<f64>,  speed: f64) -> RobotResult<()>;
        // }
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
        py_mathods! {
            fn move_linear(&self, target: &[f64; 6], speed: f64) -> RobotResult<()>;
            fn move_linear_async(&self, target: &[f64; 6], speed: f64) -> RobotResult<()>;
            fn move_linear_rel(&self, target: &[f64; 6], speed: f64) -> RobotResult<()>;
            fn move_linear_rel_async(&self, target: &[f64; 6], speed: f64) -> RobotResult<()>;
        }
        // fn move_path_prepare
        // fn move_path_start
        py_mathods! {
            fn move_path_prepare_from_file(&self, file_path: &str) -> RobotResult<()>;
            fn move_path_start(&self, path: &[f64; 6]) -> RobotResult<()>;
        }
    };
}

#[macro_export]
macro_rules! py_arm_streaming_handle {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {};
}

#[macro_export]
macro_rules! py_arm_streaming_motion {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {};
}

#[macro_export]
macro_rules! py_arm_streaming_motion_ext {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {};
}

#[macro_export]
macro_rules! py_arm_real_time_control {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {};
}

#[macro_export]
macro_rules! py_arm_real_time_control_ext {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {};
}

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

#[cfg(test)]
mod test {}

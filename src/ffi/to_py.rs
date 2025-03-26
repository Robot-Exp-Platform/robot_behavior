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

py_arm_state!(PyArmState1, 1);
py_arm_state!(PyArmState2, 2);
py_arm_state!(PyArmState3, 3);
py_arm_state!(PyArmState4, 4);
py_arm_state!(PyArmState5, 5);
py_arm_state!(PyArmState6, 6);
py_arm_state!(PyArmState7, 7);

use crate::{Pose, RobotException};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CxxPoseKind {
    Euler,
    Quat,
    Homo,
    AxisAngle,
    Position,
}

#[derive(Debug, Clone)]
pub struct CxxPoseData {
    pub kind: CxxPoseKind,
    pub values: Vec<f64>,
}

impl From<Pose> for CxxPoseData {
    fn from(value: Pose) -> Self {
        match value {
            Pose::Euler(tran, rot) => {
                let mut values = Vec::with_capacity(6);
                values.extend_from_slice(&tran);
                values.extend_from_slice(&rot);
                Self { kind: CxxPoseKind::Euler, values }
            }
            Pose::Quat(pose) => {
                let mut values = Vec::with_capacity(7);
                values.extend_from_slice(pose.translation.vector.as_slice());
                values.extend_from_slice(pose.rotation.coords.as_slice());
                Self { kind: CxxPoseKind::Quat, values }
            }
            Pose::Homo(pose) => Self { kind: CxxPoseKind::Homo, values: pose.to_vec() },
            Pose::AxisAngle(tran, axis, angle) => {
                let mut values = Vec::with_capacity(7);
                values.extend_from_slice(&tran);
                values.extend_from_slice(&axis);
                values.push(angle);
                Self { kind: CxxPoseKind::AxisAngle, values }
            }
            Pose::Position(tran) => Self { kind: CxxPoseKind::Position, values: tran.to_vec() },
        }
    }
}

impl TryFrom<CxxPoseData> for Pose {
    type Error = RobotException;

    fn try_from(value: CxxPoseData) -> Result<Self, Self::Error> {
        match value.kind {
            CxxPoseKind::Euler if value.values.len() == 6 => Ok(Pose::Euler(
                value.values[..3].try_into().unwrap(),
                value.values[3..6].try_into().unwrap(),
            )),
            CxxPoseKind::Quat if value.values.len() == 7 => Ok(Pose::from([
                value.values[0],
                value.values[1],
                value.values[2],
                value.values[3],
                value.values[4],
                value.values[5],
                value.values[6],
            ])),
            CxxPoseKind::Homo if value.values.len() == 16 => {
                Ok(Pose::Homo(value.values.try_into().unwrap()))
            }
            CxxPoseKind::AxisAngle if value.values.len() == 7 => Ok(Pose::AxisAngle(
                value.values[..3].try_into().unwrap(),
                value.values[3..6].try_into().unwrap(),
                value.values[6],
            )),
            CxxPoseKind::Position if value.values.len() == 3 => {
                Ok(Pose::Position(value.values.try_into().unwrap()))
            }
            _ => Err(RobotException::InvalidFFIData(
                "invalid CxxPoseData length for pose kind".into(),
            )),
        }
    }
}

#[macro_export]
macro_rules! cxx_robot_api {
    ($ty:ty) => {
        fn version() -> String {
            <$ty as $crate::Robot>::version()
        }

        fn init(&mut self) -> $crate::RobotResult<()> {
            <$ty as $crate::Robot>::init(self)
        }

        fn enable(&mut self) -> $crate::RobotResult<()> {
            <$ty as $crate::Robot>::enable(self)
        }

        fn disable(&mut self) -> $crate::RobotResult<()> {
            <$ty as $crate::Robot>::disable(self)
        }

        fn shutdown(&mut self) -> $crate::RobotResult<()> {
            <$ty as $crate::Robot>::shutdown(self)
        }

        fn reset(&mut self) -> $crate::RobotResult<()> {
            <$ty as $crate::Robot>::reset(self)
        }

        fn stop(&mut self) -> $crate::RobotResult<()> {
            <$ty as $crate::Robot>::stop(self)
        }

        fn emergency_stop(&mut self) -> $crate::RobotResult<()> {
            <$ty as $crate::Robot>::emergency_stop(self)
        }

        fn waiting_for_finish(&mut self) -> $crate::RobotResult<()> {
            <$ty as $crate::Robot>::waiting_for_finish(self)
        }

        fn is_moving(&mut self) -> $crate::RobotResult<bool> {
            <$ty as $crate::Robot>::is_moving(self)
        }
    };
}

#[macro_export]
macro_rules! cxx_arm_motion_api {
    ($ty:ty, $dof:expr) => {
        fn move_joint(&mut self, target: [f64; $dof]) -> $crate::RobotResult<()> {
            <$ty as $crate::MoveTo<$crate::JointSpace<$dof>>>::move_to(self, target)
        }

        fn move_joint_sync(&mut self, target: [f64; $dof]) -> $crate::RobotResult<()> {
            <$ty as $crate::MoveTo<$crate::JointSpace<$dof>>>::move_to(self, target)
        }

        fn move_flange(
            &mut self,
            target: $crate::ffi::to_cxx::CxxPoseData,
        ) -> $crate::RobotResult<()> {
            let target = $crate::Pose::try_from(target)?;
            <$ty as $crate::MoveTo<$crate::FlangeSpace>>::move_to(self, target)
        }

        fn move_flange_sync(
            &mut self,
            target: $crate::ffi::to_cxx::CxxPoseData,
        ) -> $crate::RobotResult<()> {
            let target = $crate::Pose::try_from(target)?;
            <$ty as $crate::MoveTo<$crate::FlangeSpace>>::move_to(self, target)
        }
    };
}

#[macro_export]
macro_rules! cxx_arm_bridge {
    ($mod_name:ident, $wrapper:ident($inner:ty), dof = $dof:expr) => {
        #[cxx::bridge]
        pub mod $mod_name {
            enum CxxPoseKind {
                Euler,
                Quat,
                Homo,
                AxisAngle,
                Position,
            }

            struct CxxPoseData {
                kind: CxxPoseKind,
                values: Vec<f64>,
            }

            extern "Rust" {
                type $wrapper;

                fn version() -> String;
                fn init(&mut self) -> Result<()>;
                fn enable(&mut self) -> Result<()>;
                fn disable(&mut self) -> Result<()>;
                fn shutdown(&mut self) -> Result<()>;
                fn reset(&mut self) -> Result<()>;
                fn stop(&mut self) -> Result<()>;
                fn emergency_stop(&mut self) -> Result<()>;
                fn waiting_for_finish(&mut self) -> Result<()>;
                fn is_moving(&mut self) -> Result<bool>;

                fn state(&mut self) -> Result<String>;
                fn set_load(&mut self, m: f64, x: [f64; 3], i: [f64; 9]) -> Result<()>;
                fn get_joint(&self) -> [f64; $dof];
                fn get_endpoint(&self) -> CxxPoseData;

                fn move_joint(&mut self, target: [f64; $dof]) -> Result<()>;
                fn move_joint_sync(&mut self, target: [f64; $dof]) -> Result<()>;
                fn move_flange(&mut self, target: CxxPoseData) -> Result<()>;
                fn move_flange_sync(&mut self, target: CxxPoseData) -> Result<()>;
            }
        }

        impl $wrapper {
            fn version() -> String {
                <$inner as $crate::Robot>::version()
            }

            fn init(&mut self) -> $crate::RobotResult<()> {
                <$inner as $crate::Robot>::init(&mut self.0)
            }

            fn enable(&mut self) -> $crate::RobotResult<()> {
                <$inner as $crate::Robot>::enable(&mut self.0)
            }

            fn disable(&mut self) -> $crate::RobotResult<()> {
                <$inner as $crate::Robot>::disable(&mut self.0)
            }

            fn shutdown(&mut self) -> $crate::RobotResult<()> {
                <$inner as $crate::Robot>::shutdown(&mut self.0)
            }

            fn reset(&mut self) -> $crate::RobotResult<()> {
                <$inner as $crate::Robot>::reset(&mut self.0)
            }

            fn stop(&mut self) -> $crate::RobotResult<()> {
                <$inner as $crate::Robot>::stop(&mut self.0)
            }

            fn emergency_stop(&mut self) -> $crate::RobotResult<()> {
                <$inner as $crate::Robot>::emergency_stop(&mut self.0)
            }

            fn waiting_for_finish(&mut self) -> $crate::RobotResult<()> {
                <$inner as $crate::Robot>::waiting_for_finish(&mut self.0)
            }

            fn is_moving(&mut self) -> $crate::RobotResult<bool> {
                <$inner as $crate::Robot>::is_moving(&mut self.0)
            }

            fn state(&mut self) -> $crate::RobotResult<String> {
                let state = <$inner as $crate::Arm<$dof>>::state(&mut self.0)?;
                Ok(format!("{state:?}"))
            }

            fn set_load(&mut self, m: f64, x: [f64; 3], i: [f64; 9]) -> $crate::RobotResult<()> {
                <$inner as $crate::Arm<$dof>>::set_load(&mut self.0, $crate::LoadState { m, x, i })
            }

            fn get_joint(&self) -> [f64; $dof] {
                <$inner as $crate::Arm<$dof>>::get_joint(&self.0)
            }

            fn get_endpoint(&self) -> $mod_name::CxxPoseData {
                let pose = <$inner as $crate::Arm<$dof>>::get_endpoint(&self.0);
                match pose {
                    $crate::Pose::Euler(tran, rot) => {
                        let mut values = Vec::with_capacity(6);
                        values.extend_from_slice(&tran);
                        values.extend_from_slice(&rot);
                        $mod_name::CxxPoseData { kind: $mod_name::CxxPoseKind::Euler, values }
                    }
                    $crate::Pose::Quat(pose) => {
                        let mut values = Vec::with_capacity(7);
                        values.extend_from_slice(pose.translation.vector.as_slice());
                        values.extend_from_slice(pose.rotation.coords.as_slice());
                        $mod_name::CxxPoseData { kind: $mod_name::CxxPoseKind::Quat, values }
                    }
                    $crate::Pose::Homo(pose) => $mod_name::CxxPoseData {
                        kind: $mod_name::CxxPoseKind::Homo,
                        values: pose.to_vec(),
                    },
                    $crate::Pose::AxisAngle(tran, axis, angle) => {
                        let mut values = Vec::with_capacity(7);
                        values.extend_from_slice(&tran);
                        values.extend_from_slice(&axis);
                        values.push(angle);
                        $mod_name::CxxPoseData { kind: $mod_name::CxxPoseKind::AxisAngle, values }
                    }
                    $crate::Pose::Position(tran) => $mod_name::CxxPoseData {
                        kind: $mod_name::CxxPoseKind::Position,
                        values: tran.to_vec(),
                    },
                }
            }

            fn move_joint(&mut self, target: [f64; $dof]) -> $crate::RobotResult<()> {
                <$inner as $crate::MoveTo<$crate::JointSpace<$dof>>>::move_to(&mut self.0, target)
            }

            fn move_joint_sync(&mut self, target: [f64; $dof]) -> $crate::RobotResult<()> {
                <$inner as $crate::MoveTo<$crate::JointSpace<$dof>>>::move_to(&mut self.0, target)
            }

            fn move_flange(&mut self, target: $mod_name::CxxPoseData) -> $crate::RobotResult<()> {
                let target = Self::cxx_pose_to_pose(target)?;
                <$inner as $crate::MoveTo<$crate::FlangeSpace>>::move_to(&mut self.0, target)
            }

            fn move_flange_sync(
                &mut self,
                target: $mod_name::CxxPoseData,
            ) -> $crate::RobotResult<()> {
                let target = Self::cxx_pose_to_pose(target)?;
                <$inner as $crate::MoveTo<$crate::FlangeSpace>>::move_to(&mut self.0, target)
            }

            fn cxx_pose_to_pose(
                target: $mod_name::CxxPoseData,
            ) -> $crate::RobotResult<$crate::Pose> {
                match target.kind {
                    $mod_name::CxxPoseKind::Euler if target.values.len() == 6 => {
                        Ok($crate::Pose::Euler(
                            target.values[..3].try_into().unwrap(),
                            target.values[3..6].try_into().unwrap(),
                        ))
                    }
                    $mod_name::CxxPoseKind::Quat if target.values.len() == 7 => {
                        Ok($crate::Pose::from([
                            target.values[0],
                            target.values[1],
                            target.values[2],
                            target.values[3],
                            target.values[4],
                            target.values[5],
                            target.values[6],
                        ]))
                    }
                    $mod_name::CxxPoseKind::Homo if target.values.len() == 16 => {
                        Ok($crate::Pose::Homo(target.values.try_into().unwrap()))
                    }
                    $mod_name::CxxPoseKind::AxisAngle if target.values.len() == 7 => {
                        Ok($crate::Pose::AxisAngle(
                            target.values[..3].try_into().unwrap(),
                            target.values[3..6].try_into().unwrap(),
                            target.values[6],
                        ))
                    }
                    $mod_name::CxxPoseKind::Position if target.values.len() == 3 => {
                        Ok($crate::Pose::Position(target.values.try_into().unwrap()))
                    }
                    _ => Err($crate::RobotException::InvalidFFIData(
                        "invalid CxxPoseData length for pose kind".into(),
                    )),
                }
            }
        }
    };
}

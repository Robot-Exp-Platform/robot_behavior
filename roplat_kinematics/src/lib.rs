//! Denavit–Hartenberg kinematics for `roplat_behavior` arms.
//!
//! An arm type declares its [`DhParam`] table through [`ArmForwardKinematics`]
//! and, with joint limits, gets the iterative solvers of
//! [`ArmInverseKinematics`]. [`dh_kinematics!`] connects these to the
//! `roplat_behavior` model maps (`SpaceMap` for flange pose, Jacobian and,
//! optionally, inverse kinematics), so generic code can use the type through
//! `ForwardKinematics`, `JacobianModel` and `InverseKinematics`.

#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

mod dh;
mod kinematics;

pub use dh::DhParam;
pub use kinematics::*;

#[doc(hidden)]
pub mod __private {
    pub use roplat_behavior;
}

/// Concatenate two fixed-size arrays into a single `[T; N + M]`.
///
/// # Example
/// ```
/// #![allow(incomplete_features)]
/// #![feature(generic_const_exprs)]
/// use roplat_kinematics::combine_array;
///
/// assert_eq!(combine_array(&[1, 2, 3], &[4, 5]), [1, 2, 3, 4, 5]);
/// ```
pub fn combine_array<T: Copy, const N: usize, const M: usize>(
    v1: &[T; N],
    v2: &[T; M],
) -> [T; N + M] {
    let mut result = [v1[0]; N + M];
    result[..N].copy_from_slice(v1);
    result[N..].copy_from_slice(v2);
    result
}

/// Implements the `roplat_behavior` model maps of a Denavit–Hartenberg arm.
///
/// `dh_kinematics!(Arm, N)` implements `SpaceMap<JointSpace<N>, FlangeSpace>`
/// (forward kinematics) and `SpaceMap<JointSpace<N>, JacobianSpace<N>>` for a
/// type implementing [`ArmForwardKinematics<N>`]. Adding `inverse` also
/// implements `SpaceMap<FlangeSpace, JointSpace<N>>` through
/// [`ArmInverseKinematics::ik_solve`], seeded with `Joints::JOINT_DEFAULT`.
///
/// Before 0.9 these maps were blanket impls in `robot_behavior`; a macro keeps
/// the contract crate independent of this one.
///
/// ```
/// #![allow(incomplete_features)]
/// #![feature(generic_const_exprs)]
/// use roplat_behavior::{ForwardKinematics, Joints};
/// use roplat_kinematics::{ArmForwardKinematics, DhParam, dh_kinematics};
///
/// struct Planar;
/// impl ArmForwardKinematics<2> for Planar {
///     const DH: [DhParam; 2] = [
///         DhParam::DH { theta: 0., d: 0., r: 1., alpha: 0. },
///         DhParam::DH { theta: 0., d: 0., r: 1., alpha: 0. },
///     ];
/// }
/// dh_kinematics!(Planar, 2);
///
/// let pose = Planar.forward_kinematics([0., 0.]).unwrap();
/// assert!((pose.position()[0] - 2.).abs() < 1e-12);
/// ```
#[macro_export]
macro_rules! dh_kinematics {
    ($ty:ty, $n:expr) => {
        impl
            $crate::__private::roplat_behavior::SpaceMap<
                $crate::__private::roplat_behavior::JointSpace<$n>,
                $crate::__private::roplat_behavior::FlangeSpace,
            > for $ty
        {
            type Input = [f64; $n];
            type Output = $crate::__private::roplat_behavior::Pose;

            fn map(
                &self,
                input: Self::Input,
            ) -> $crate::__private::roplat_behavior::RobotResult<Self::Output> {
                Ok(<$ty as $crate::ArmForwardKinematics<$n>>::fk_end_pose(
                    &input,
                ))
            }
        }

        impl
            $crate::__private::roplat_behavior::SpaceMap<
                $crate::__private::roplat_behavior::JointSpace<$n>,
                $crate::__private::roplat_behavior::JacobianSpace<$n>,
            > for $ty
        {
            type Input = [f64; $n];
            type Output = $crate::__private::roplat_behavior::Jaco<$n>;

            fn map(
                &self,
                input: Self::Input,
            ) -> $crate::__private::roplat_behavior::RobotResult<Self::Output> {
                Ok(
                    <$ty as $crate::ArmForwardKinematics<$n>>::kine_cache(&input, &[0.; $n])
                        .jacobian(),
                )
            }
        }
    };
    ($ty:ty, $n:expr, inverse) => {
        $crate::dh_kinematics!($ty, $n);

        impl
            $crate::__private::roplat_behavior::SpaceMap<
                $crate::__private::roplat_behavior::FlangeSpace,
                $crate::__private::roplat_behavior::JointSpace<$n>,
            > for $ty
        {
            type Input = $crate::__private::roplat_behavior::Pose;
            type Output = [f64; $n];

            fn map(
                &self,
                input: Self::Input,
            ) -> $crate::__private::roplat_behavior::RobotResult<Self::Output> {
                let q0 = $crate::__private::roplat_behavior::JVec::<$n>::from_row_slice(
                    &<$ty as $crate::__private::roplat_behavior::Joints<$n>>::JOINT_DEFAULT,
                );
                let q = <$ty as $crate::ArmInverseKinematics<$n>>::ik_solve(
                    &q0,
                    &input,
                    Default::default(),
                );
                Ok(q.as_slice().try_into().unwrap())
            }
        }
    };
}

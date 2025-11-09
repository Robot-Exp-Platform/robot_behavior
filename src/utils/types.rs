use nalgebra as na;

pub fn homo_to_isometry(homo: &[f64; 16]) -> na::Isometry3<f64> {
    let rot = na::Rotation3::from_matrix(
        &na::Matrix4::from_column_slice(homo)
            .remove_column(3)
            .remove_row(3),
    );
    na::Isometry3::from_parts(
        na::Vector3::new(homo[12], homo[13], homo[14]).into(),
        rot.into(),
    )
}

pub fn combine_array<T: Copy, const N: usize, const M: usize>(
    v1: &[T; N],
    v2: &[T; M],
) -> [T; N + M] {
    let mut result = [v1[0]; N + M];
    result[..N].copy_from_slice(v1);
    result[N..].copy_from_slice(v2);
    result
}

pub fn rad_to_deg<const N: usize>(arr: [f64; N]) -> [f64; N] {
    arr.map(|x| x.to_degrees())
}

/// Build an `Isometry3` from position array and orientation quaternion in XYZW order.
pub fn isometry_from_raw_parts(
    position: [f64; 3],
    orientation_xyzw: [f64; 4],
) -> na::Isometry3<f64> {
    let translation = na::Translation3::new(position[0], position[1], position[2]);
    let quaternion = na::Quaternion::new(
        orientation_xyzw[3],
        orientation_xyzw[0],
        orientation_xyzw[1],
        orientation_xyzw[2],
    );
    let rotation = na::UnitQuaternion::from_quaternion(quaternion);
    na::Isometry3::from_parts(translation, rotation)
}

/// Build an `Isometry3` from a `[position, orientation]` frame slice.
pub fn isometry_from_frame(frame: &[f64; 7]) -> na::Isometry3<f64> {
    isometry_from_raw_parts(
        [frame[0], frame[1], frame[2]],
        [frame[3], frame[4], frame[5], frame[6]],
    )
}

/// Extract translation (XYZ) and quaternion (XYZW) components from an `Isometry3`.
pub fn isometry_to_raw_parts(transform: &na::Isometry3<f64>) -> ([f64; 3], [f64; 4]) {
    let rotation = transform.rotation;
    (
        transform.translation.into(),
        [rotation.i, rotation.j, rotation.k, rotation.w],
    )
}

/// Write an `Isometry3` into a `[position, orientation]` frame buffer.
pub fn isometry_write_to_frame(transform: &na::Isometry3<f64>, frame: &mut [f64; 7]) {
    let (position, orientation) = isometry_to_raw_parts(transform);
    frame[..3].copy_from_slice(&position);
    frame[3..7].copy_from_slice(&orientation);
}

pub const fn to_radians_array<const N: usize>(arr: [f64; N]) -> [f64; N] {
    let mut result = [0.0; N];
    let mut i = 0;
    while i < N {
        result[i] = arr[i].to_radians();
        i += 1;
    }
    result
}

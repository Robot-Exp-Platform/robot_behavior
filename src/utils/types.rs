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

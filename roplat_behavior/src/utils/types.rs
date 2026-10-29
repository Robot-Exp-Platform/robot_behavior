/// Convert an array of angles from degrees to radians, element-wise.
///
/// `const fn` so it can build joint-limit constants at compile time, e.g. in a
/// [`Joints`](crate::Joints) impl.
pub const fn to_radians_array<const N: usize>(arr: [f64; N]) -> [f64; N] {
    let mut result = [0.0; N];
    let mut i = 0;
    while i < N {
        result[i] = arr[i].to_radians();
        i += 1;
    }
    result
}

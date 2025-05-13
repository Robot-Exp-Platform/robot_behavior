use std::simd::{LaneCount, Simd, SupportedLaneCount};

#[inline(always)]
pub fn limit<'a, const N: usize>(q: &'a mut [f64; N], q_bound: &[f64; N]) -> &'a mut [f64; N] {
    q.iter_mut().zip(q_bound.iter()).for_each(|(qi, &qbi)| {
        *qi = (*qi).clamp(-qbi, qbi);
    });
    q
}

#[inline(always)]
pub fn clamp<'a, const N: usize>(
    q: &'a mut [f64; N],
    min: &[f64; N],
    max: &[f64; N],
) -> &'a mut [f64; N] {
    q.iter_mut()
        .zip(min.iter().zip(max.iter())) // 合并为单次 zip
        .for_each(|(qi, (&min, &max))| {
            *qi = qi.clamp(min, max);
        });
    q
}

#[inline(always)]
pub fn limit_dot<'a, const N: usize>(
    q: &'a mut [f64; N],
    q_last: &[f64; N],
    time: f64,
    q_dot_bound: &[f64; N],
) -> &'a mut [f64; N] {
    q.iter_mut()
        .zip(q_last.iter())
        .zip(q_dot_bound.iter())
        .for_each(|((qi, &qli), &qbi)| {
            *qi = (*qi).clamp(qli - qbi * time, qli + qbi * time);
        });
    q
}

#[inline(always)]
pub fn difference<const N: usize>(q: &[f64; N], q_last: &[f64; N], time: f64) -> [f64; N] {
    let inv_time = 1.0 / time;
    let mut q_diff = [0.0; N];
    q_diff
        .iter_mut()
        .zip(q.iter().zip(q_last.iter()))
        .for_each(|(qi, (&qi_val, &qli_val))| {
            *qi = (qi_val - qli_val) * inv_time;
        });
    q_diff
}

#[inline(always)]
pub fn difference_simd<const N: usize>(
    q: Simd<f64, N>,
    q_last: Simd<f64, N>,
    time: Simd<f64, N>,
) -> Simd<f64, N>
where
    LaneCount<N>: SupportedLaneCount,
{
    (q - q_last) / time
}

#[inline(always)]
pub fn update<'a, const N: usize>(
    q: &'a mut [f64; N],
    q_dot: &[f64; N],
    time: f64,
) -> &'a mut [f64; N] {
    q.iter_mut().zip(q_dot.iter()).for_each(|(qi, &qdi)| {
        *qi += qdi * time;
    });
    q
}

#[inline(always)]
pub fn update_simd<const N: usize>(q: [f64; N], q_dot: &[f64; N], time: f64) -> [f64; N]
where
    LaneCount<N>: SupportedLaneCount,
{
    let q = Simd::from_array(q);
    let q_dot = Simd::from_array(*q_dot);
    let time = Simd::splat(time);
    (q + q_dot * time).to_array()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simd_optimization() {
        let mut q = [1.5; 8];
        let q_last = [0.0; 8];
        let time = 0.1;

        let start_time = std::time::Instant::now();
        for _ in 0..100_000 {
            let _ = difference(&mut q, &q_last, time);
        }
        println!("difference: {:?}", start_time.elapsed());
        let q = Simd::from_array(q);
        let q_last = Simd::from_array(q_last);
        let time = Simd::splat(time);

        let start_time = std::time::Instant::now();
        for _ in 0..100_000 {
            let _ = difference_simd(q, q_last, time);
        }
        println!("difference_simd: {:?}", start_time.elapsed());
    }
}

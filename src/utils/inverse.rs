use crate::{DhParam, Pose};

pub fn ik_planar_2r_all(dh: &[DhParam; 2], target: &Pose) -> Option<Vec<[f64; 2]>> {
    let [l1, l2] = dh.map(|d| match d {
        DhParam::DH { r: a, .. } | DhParam::MDH { a, .. } => a,
    });
    let [x, y, z] = target.position();

    if z.abs() > 1e-6 {
        return None;
    }

    let cos_2 = ((x * x + y * y - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)).clamp(-1.0, 1.0);
    let sin_2 = (1.0 - cos_2 * cos_2).sqrt();

    let q2_candidates = [sin_2.atan2(cos_2), (-sin_2).atan2(cos_2)];
    let mut solutions = Vec::with_capacity(2);
    for q2 in q2_candidates.iter().copied() {
        // 对每个候选角度单独使用其正弦/余弦，避免固定正弦符号带来的错误
        let k1 = l1 + l2 * q2.cos();
        let k2 = l2 * q2.sin();
        let q1 = y.atan2(x) - k2.atan2(k1);
        solutions.push([q1, q2]);
    }

    Some(solutions)
}

#[cfg(test)]
mod test {
    use std::f64::consts::FRAC_PI_2;

    use super::*;

    #[test]
    fn test_ik_planar_2r() {
        let dh = [DhParam::DH { theta: 0.0, d: 0.0, r: 1.0, alpha: 0.0 }; 2];
        let target = Pose::Position([1.0, 1.0, 0.0]);
        let solutions = ik_planar_2r_all(&dh, &target).unwrap();
        assert_eq!(solutions.len(), 2);
        println!("{:?}", solutions);
        assert_eq!(solutions[0], [0., FRAC_PI_2]);
        assert_eq!(solutions[1], [FRAC_PI_2, -FRAC_PI_2]);
    }
}

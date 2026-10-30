//! A hash grid over points, for neighbour queries.

use std::collections::HashMap;

pub(crate) struct Grid<'a> {
    points: &'a [[f64; 3]],
    cell: f64,
    cells: HashMap<[i64; 3], Vec<usize>>,
}

impl<'a> Grid<'a> {
    /// The finite `points` in cells of edge `cell`.
    pub(crate) fn new(points: &'a [[f64; 3]], cell: f64) -> Self {
        let mut grid = Self { points, cell, cells: HashMap::new() };
        for (i, p) in points.iter().enumerate() {
            if is_finite(p) {
                grid.cells.entry(grid.key(p)).or_default().push(i);
            }
        }
        grid
    }

    fn key(&self, p: &[f64; 3]) -> [i64; 3] {
        p.map(|c| (c / self.cell).floor() as i64)
    }

    /// Calls `visit` with each point within `radius` of `p`, `p` itself
    /// included if it is one of the points.
    pub(crate) fn within(&self, p: &[f64; 3], radius: f64, mut visit: impl FnMut(usize)) {
        if !is_finite(p) {
            return;
        }
        let span = (radius / self.cell).ceil() as i64;
        let key = self.key(p);
        for dx in -span..=span {
            for dy in -span..=span {
                for dz in -span..=span {
                    let Some(cell) = self.cells.get(&[key[0] + dx, key[1] + dy, key[2] + dz])
                    else {
                        continue;
                    };
                    for &i in cell {
                        if squared_distance(&self.points[i], p) <= radius * radius {
                            visit(i);
                        }
                    }
                }
            }
        }
    }

    /// The point nearest to `p` within `radius`, and its squared distance.
    pub(crate) fn nearest(&self, p: &[f64; 3], radius: f64) -> Option<(usize, f64)> {
        let mut nearest: Option<(usize, f64)> = None;
        self.within(p, radius, |i| {
            let d = squared_distance(&self.points[i], p);
            if nearest.is_none_or(|(_, best)| d < best) {
                nearest = Some((i, d));
            }
        });
        nearest
    }
}

pub(crate) fn is_finite(p: &[f64; 3]) -> bool {
    p.iter().all(|c| c.is_finite())
}

pub(crate) fn squared_distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    (0..3).map(|k| (a[k] - b[k]).powi(2)).sum()
}

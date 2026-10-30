//! Calibration targets in images: where a chessboard's inner corners
//! appear ([`detect_chessboard`]), and where a planar target is relative to
//! the camera ([`target_pose`]), the `target` of a
//! [`HandEyeSample`](crate::calibration::HandEyeSample).
//!
//! The detector finds the X-shaped corners where four squares meet:
//!
//! 1. A saddle response on the slightly blurred image, `I_xy² − I_xx I_yy`,
//!    which is large where intensity rises along one diagonal and falls
//!    along the other.
//! 2. Its local maxima, kept only where a ring around them alternates light
//!    and dark four times, which rejects edges, blobs and the L-shaped
//!    corners at the board's border.
//! 3. Subpixel positions by the gradient condition OpenCV's
//!    `cornerSubPix` uses: at the corner, every nearby gradient is
//!    orthogonal to the direction to the corner.
//! 4. A lattice grown from a seed corner and its two nearest neighbours,
//!    predicting each next corner from the last step along a row or column,
//!    so perspective is followed. It must come out exactly
//!    `columns × rows`.
//! 5. An order: rows of `columns` corners, the board's `x` along a row and
//!    `y` down the columns, `z = x × y` pointing away from the camera, and
//!    the first corner beside the board's dark corner square. A board whose
//!    `columns + rows` is odd has one dark and one light corner square at
//!    opposite ends (OpenCV's 9 × 6 board does), so the order is the same
//!    from every view; with an even sum it is fixed only up to a half turn.
//!
//! [`target_pose`] then solves the perspective-n-point problem for the
//! planar board: a homography gives the first pose, refined by
//! Levenberg–Marquardt on the reprojection error.
//!
//! Images must be rectified: lens distortion is not modelled here. Rectify
//! raw images with [`lens::Rectify`](crate::lens::Rectify) first, or
//! undistort the detected corners with [`lens::undistort`](crate::lens::undistort)
//! and solve the pose with [`lens::rectified`](crate::lens::rectified)`(info)`.
//!
//! ```
//! use roplat_frames::target::Chessboard;
//!
//! let board = Chessboard { columns: 9, rows: 6, square: 0.025 };
//! let corners = board.corners();
//! assert_eq!(corners.len(), 54);
//! assert_eq!(corners[10], [0.025, 0.025, 0.0]);
//! ```

use nalgebra as na;
use roplat_msgs::CameraInfo;

/// A chessboard by its inner corners, where four squares meet.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Chessboard {
    /// Inner corners along a row: the board's `x`.
    pub columns: usize,
    /// Inner corners down a column: the board's `y`.
    pub rows: usize,
    /// Square edge (m).
    pub square: f64,
}

impl Chessboard {
    /// The inner corners in the board frame, in detection order: corner
    /// `j * columns + i` is at `(i square, j square, 0)`.
    pub fn corners(&self) -> Vec<[f64; 3]> {
        (0..self.rows)
            .flat_map(|j| {
                (0..self.columns)
                    .map(move |i| [i as f64 * self.square, j as f64 * self.square, 0.0])
            })
            .collect()
    }
}

/// Settings for [`detect_chessboard`].
#[derive(Clone, Copy, Debug)]
pub struct DetectOptions {
    /// Gaussian blur (px) before the corner response: about 1 for a sharp
    /// image, more for a noisy one.
    pub blur: f64,
    /// Window radius (px) for local maxima, the light-dark ring and
    /// subpixel refinement. Keep it under half the smallest square in the
    /// image.
    pub radius: usize,
    /// Least corner response, as a share of the strongest one.
    pub threshold: f64,
}

impl Default for DetectOptions {
    fn default() -> Self {
        Self { blur: 1.0, radius: 4, threshold: 0.05 }
    }
}

/// The inner corners of `board` in an 8-bit grayscale image, row-major
/// `width × height`, in the order of [`Chessboard::corners`]; see the
/// [module](self) docs. `None` unless all of them are found.
pub fn detect_chessboard(
    image: &[u8],
    width: usize,
    height: usize,
    board: &Chessboard,
    options: &DetectOptions,
) -> Option<Vec<[f64; 2]>> {
    if board.columns < 2 || board.rows < 2 || image.len() < width * height {
        return None;
    }
    let gray = Gray::new(image, width, height).blurred(options.blur);
    let candidates = corner_candidates(&gray, options);
    let corners: Vec<na::Point2<f64>> = candidates
        .iter()
        .filter_map(|&(x, y)| refine(&gray, na::Point2::new(x as f64, y as f64), options.radius))
        .collect();
    let corners = dedup(corners, options.radius as f64 / 2.0);
    // Seeds in turn until one grows into the whole board.
    corners.iter().enumerate().find_map(|(seed, _)| {
        let grid = grow(&corners, seed)?;
        order(&gray, &corners, grid, board)
    })
}

/// A planar target's pose in the camera's optical frame, with the
/// root-mean-square reprojection error (px).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TargetPose {
    /// The target frame in the camera's optical frame.
    pub pose: na::Isometry3<f64>,
    pub rms: f64,
}

/// The pose of a planar target from where its points appear: `pixels[k]`
/// is where `object[k]` (in the target frame, all with `z = 0`) appears,
/// through `info`'s pinhole intrinsics. `None` for fewer than four points,
/// points off the plane, or a view that determines no pose.
pub fn target_pose(
    pixels: &[[f64; 2]],
    object: &[[f64; 3]],
    info: &CameraInfo,
) -> Option<TargetPose> {
    if pixels.len() != object.len() || pixels.len() < 4 || object.iter().any(|p| p[2].abs() > 1e-9)
    {
        return None;
    }
    let camera = Pinhole::new(info)?;
    let initial = homography_pose(pixels, object, &camera)?;
    let (pose, rms) = refine_pose(initial, pixels, object, &camera);
    (rms.is_finite()).then_some(TargetPose { pose, rms })
}

/// An 8-bit image as floats, read with clamping at the border.
struct Gray {
    width: usize,
    height: usize,
    values: Vec<f64>,
}

impl Gray {
    fn new(image: &[u8], width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            values: image[..width * height].iter().map(|&v| v as f64).collect(),
        }
    }

    fn at(&self, x: isize, y: isize) -> f64 {
        let x = x.clamp(0, self.width as isize - 1) as usize;
        let y = y.clamp(0, self.height as isize - 1) as usize;
        self.values[y * self.width + x]
    }

    /// Bilinear interpolation.
    fn sample(&self, x: f64, y: f64) -> f64 {
        let (x0, y0) = (x.floor(), y.floor());
        let (fx, fy) = (x - x0, y - y0);
        let (x0, y0) = (x0 as isize, y0 as isize);
        let top = self.at(x0, y0) * (1.0 - fx) + self.at(x0 + 1, y0) * fx;
        let bottom = self.at(x0, y0 + 1) * (1.0 - fx) + self.at(x0 + 1, y0 + 1) * fx;
        top * (1.0 - fy) + bottom * fy
    }

    /// Separable Gaussian blur of standard deviation `sigma` (px).
    fn blurred(self, sigma: f64) -> Self {
        if sigma <= 0.0 {
            return self;
        }
        let radius = (3.0 * sigma).ceil() as isize;
        let kernel: Vec<f64> = (-radius..=radius)
            .map(|k| (-(k * k) as f64 / (2.0 * sigma * sigma)).exp())
            .collect();
        let total: f64 = kernel.iter().sum();
        let pass = |source: &Gray, along_x: bool| {
            let mut values = vec![0.0; source.values.len()];
            for y in 0..source.height as isize {
                for x in 0..source.width as isize {
                    let sum: f64 = (-radius..=radius)
                        .zip(&kernel)
                        .map(|(k, w)| {
                            w * if along_x {
                                source.at(x + k, y)
                            } else {
                                source.at(x, y + k)
                            }
                        })
                        .sum();
                    values[y as usize * source.width + x as usize] = sum / total;
                }
            }
            Gray { width: source.width, height: source.height, values }
        };
        let once = pass(&self, true);
        pass(&once, false)
    }
}

/// Pixels whose saddle response is a local maximum above the threshold
/// and whose ring alternates light and dark four times.
fn corner_candidates(gray: &Gray, options: &DetectOptions) -> Vec<(usize, usize)> {
    let (w, h) = (gray.width, gray.height);
    let mut response = vec![0.0; w * h];
    for y in 1..h.saturating_sub(1) {
        for x in 1..w.saturating_sub(1) {
            let (xi, yi) = (x as isize, y as isize);
            let centre = gray.at(xi, yi);
            let ixx = gray.at(xi + 1, yi) - 2.0 * centre + gray.at(xi - 1, yi);
            let iyy = gray.at(xi, yi + 1) - 2.0 * centre + gray.at(xi, yi - 1);
            let ixy = (gray.at(xi + 1, yi + 1) - gray.at(xi + 1, yi - 1) - gray.at(xi - 1, yi + 1)
                + gray.at(xi - 1, yi - 1))
                / 4.0;
            response[y * w + x] = (ixy * ixy - ixx * iyy).max(0.0);
        }
    }
    let strongest = response.iter().copied().fold(0.0, f64::max);
    if strongest <= 0.0 {
        return Vec::new();
    }
    let floor = options.threshold * strongest;
    let r = options.radius.max(1);
    let mut candidates = Vec::new();
    for y in r..h.saturating_sub(r) {
        for x in r..w.saturating_sub(r) {
            let value = response[y * w + x];
            if value < floor {
                continue;
            }
            let is_max = (y - r..=y + r).all(|yy| {
                (x - r..=x + r).all(|xx| {
                    let other = response[yy * w + xx];
                    // Ties go to the first pixel in raster order.
                    other < value || (other == value && (yy, xx) >= (y, x))
                })
            });
            if is_max && alternates(gray, x as f64, y as f64, r as f64) {
                candidates.push((x, y));
            }
        }
    }
    candidates
}

/// Whether the ring of `radius` around `(x, y)` changes between lighter
/// and darker than its mean exactly four times, as around an X corner.
fn alternates(gray: &Gray, x: f64, y: f64, radius: f64) -> bool {
    const SAMPLES: usize = 32;
    let ring: Vec<f64> = (0..SAMPLES)
        .map(|k| {
            let angle = std::f64::consts::TAU * k as f64 / SAMPLES as f64;
            gray.sample(x + radius * angle.cos(), y + radius * angle.sin())
        })
        .collect();
    let mean = ring.iter().sum::<f64>() / SAMPLES as f64;
    let spread = ring.iter().map(|v| (v - mean).abs()).sum::<f64>() / SAMPLES as f64;
    if spread < 1.0 {
        return false;
    }
    let light: Vec<bool> = ring.iter().map(|&v| v > mean).collect();
    (0..SAMPLES)
        .filter(|&k| light[k] != light[(k + 1) % SAMPLES])
        .count()
        == 4
}

/// The corner near `start` where the window's gradients are all orthogonal
/// to the direction to it; `None` if it drifts out of the window.
fn refine(gray: &Gray, start: na::Point2<f64>, radius: usize) -> Option<na::Point2<f64>> {
    let r = radius as isize;
    let spread = radius as f64;
    let mut corner = start;
    for _ in 0..20 {
        let mut normal = na::Matrix2::zeros();
        let mut rhs = na::Vector2::zeros();
        for dy in -r..=r {
            for dx in -r..=r {
                // The blurred junction itself says nothing about the edges.
                if dx * dx + dy * dy <= 2 {
                    continue;
                }
                let p = na::Point2::new(corner.x + dx as f64, corner.y + dy as f64);
                let g = na::Vector2::new(
                    (gray.sample(p.x + 1.0, p.y) - gray.sample(p.x - 1.0, p.y)) / 2.0,
                    (gray.sample(p.x, p.y + 1.0) - gray.sample(p.x, p.y - 1.0)) / 2.0,
                );
                let weight = (-((dx * dx + dy * dy) as f64) / (2.0 * spread * spread)).exp();
                let gg = g * g.transpose() * weight;
                normal += gg;
                rhs += gg * p.coords;
            }
        }
        let next = na::Point2::from(normal.try_inverse()? * rhs);
        let moved = (next - corner).norm();
        corner = next;
        if (corner - start).norm() > radius as f64 {
            return None;
        }
        if moved < 1e-3 {
            break;
        }
    }
    Some(corner)
}

/// Merges corners closer than `distance`, keeping the first.
fn dedup(corners: Vec<na::Point2<f64>>, distance: f64) -> Vec<na::Point2<f64>> {
    let mut kept: Vec<na::Point2<f64>> = Vec::new();
    for corner in corners {
        if kept.iter().all(|k| (k - corner).norm() >= distance) {
            kept.push(corner);
        }
    }
    kept
}

type Grid = std::collections::HashMap<(i32, i32), usize>;

/// The lattice of corners grown from `seed`, by cell.
fn grow(corners: &[na::Point2<f64>], seed: usize) -> Option<Grid> {
    let origin = corners[seed];
    let mut near: Vec<usize> = (0..corners.len()).filter(|&k| k != seed).collect();
    near.sort_by(|&a, &b| {
        (corners[a] - origin)
            .norm()
            .total_cmp(&(corners[b] - origin).norm())
    });
    let &first = near.first()?;
    let u = corners[first] - origin;
    // The nearest neighbour across the first direction, not along a diagonal.
    let &second = near.iter().take(8).find(|&&k| {
        let v = corners[k] - origin;
        (u.dot(&v) / (u.norm() * v.norm())).abs() < 0.5 && v.norm() < 2.0 * u.norm()
    })?;
    let v = corners[second] - origin;
    let mut grid = Grid::from([((0, 0), seed), ((1, 0), first), ((0, 1), second)]);
    let mut used: Vec<bool> = vec![false; corners.len()];
    for &k in [seed, first, second].iter() {
        used[k] = true;
    }
    loop {
        let mut added = false;
        let cells: Vec<((i32, i32), usize)> = grid.iter().map(|(&c, &k)| (c, k)).collect();
        for ((i, j), k) in cells {
            for (di, dj) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let next = (i + di, j + dj);
                if grid.contains_key(&next) {
                    continue;
                }
                // Continue the last step along this line, else step as the
                // seed's neighbours do.
                let step = match grid.get(&(i - di, j - dj)) {
                    Some(&back) => corners[k] - corners[back],
                    None if di != 0 => u * di as f64,
                    None => v * dj as f64,
                };
                let predicted = corners[k] + step;
                let reach = 0.3 * step.norm();
                let found = (0..corners.len())
                    .filter(|&n| !used[n])
                    .map(|n| (n, (corners[n] - predicted).norm()))
                    .filter(|&(_, d)| d < reach)
                    .min_by(|a, b| a.1.total_cmp(&b.1));
                if let Some((n, _)) = found {
                    grid.insert(next, n);
                    used[n] = true;
                    added = true;
                }
            }
        }
        if !added {
            return Some(grid);
        }
    }
}

/// The corners of a complete `columns × rows` lattice in the board's order;
/// see the [module](self) docs.
fn order(
    gray: &Gray,
    corners: &[na::Point2<f64>],
    grid: Grid,
    board: &Chessboard,
) -> Option<Vec<[f64; 2]>> {
    let (imin, imax) = grid
        .keys()
        .map(|c| c.0)
        .fold((i32::MAX, i32::MIN), |(lo, hi), i| (lo.min(i), hi.max(i)));
    let (jmin, jmax) = grid
        .keys()
        .map(|c| c.1)
        .fold((i32::MAX, i32::MIN), |(lo, hi), j| (lo.min(j), hi.max(j)));
    let (a, b) = ((imax - imin + 1) as usize, (jmax - jmin + 1) as usize);
    if grid.len() != a * b {
        return None;
    }
    let at = |i: usize, j: usize| corners[grid[&(imin + i as i32, jmin + j as i32)]];
    // cells[j][i], `columns` wide.
    let mut cells: Vec<Vec<na::Point2<f64>>> = if (a, b) == (board.columns, board.rows) {
        (0..b).map(|j| (0..a).map(|i| at(i, j)).collect()).collect()
    } else if (b, a) == (board.columns, board.rows) {
        (0..a).map(|i| (0..b).map(|j| at(i, j)).collect()).collect()
    } else {
        return None;
    };
    let (columns, rows) = (board.columns, board.rows);
    // The board's z away from the camera: x then y turn clockwise on screen.
    let x = cells[0][1] - cells[0][0];
    let y = cells[1][0] - cells[0][0];
    if x.perp(&y) < 0.0 {
        cells.reverse();
    }
    // The first corner beside the dark corner square.
    let square_at = |corner: na::Point2<f64>, along: na::Vector2<f64>, down: na::Vector2<f64>| {
        let centre = corner - (along + down) / 2.0;
        gray.sample(centre.x, centre.y)
    };
    let first = square_at(
        cells[0][0],
        cells[0][1] - cells[0][0],
        cells[1][0] - cells[0][0],
    );
    let (l, m) = (rows - 1, columns - 1);
    let last = square_at(
        cells[l][m],
        cells[l][m - 1] - cells[l][m],
        cells[l - 1][m] - cells[l][m],
    );
    if first > last {
        cells.reverse();
        for row in &mut cells {
            row.reverse();
        }
    }
    Some(cells.into_iter().flatten().map(|p| [p.x, p.y]).collect())
}

/// Pinhole intrinsics.
struct Pinhole {
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
}

impl Pinhole {
    fn new(info: &CameraInfo) -> Option<Self> {
        let (fx, fy, cx, cy) = (info.k[0], info.k[4], info.k[2], info.k[5]);
        (fx > 0.0 && fy > 0.0).then_some(Self { fx, fy, cx, cy })
    }

    fn project(&self, p: &na::Point3<f64>) -> Option<na::Point2<f64>> {
        (p.z > 1e-9)
            .then(|| na::Point2::new(self.fx * p.x / p.z + self.cx, self.fy * p.y / p.z + self.cy))
    }
}

/// The pose from the homography between the board plane and the
/// normalised image (Zhang's decomposition).
fn homography_pose(
    pixels: &[[f64; 2]],
    object: &[[f64; 3]],
    camera: &Pinhole,
) -> Option<na::Isometry3<f64>> {
    let image: Vec<na::Point2<f64>> = pixels
        .iter()
        .map(|p| {
            na::Point2::new(
                (p[0] - camera.cx) / camera.fx,
                (p[1] - camera.cy) / camera.fy,
            )
        })
        .collect();
    let plane: Vec<na::Point2<f64>> = object.iter().map(|p| na::Point2::new(p[0], p[1])).collect();
    let (ti, image) = normalise(&image)?;
    let (tp, plane) = normalise(&plane)?;
    // Direct linear transform: the null vector of the stacked equations.
    let mut ata = na::SMatrix::<f64, 9, 9>::zeros();
    for (x, p) in image.iter().zip(&plane) {
        let rows = [
            na::SVector::<f64, 9>::from([
                p.x,
                p.y,
                1.0,
                0.0,
                0.0,
                0.0,
                -x.x * p.x,
                -x.x * p.y,
                -x.x,
            ]),
            na::SVector::<f64, 9>::from([
                0.0,
                0.0,
                0.0,
                p.x,
                p.y,
                1.0,
                -x.y * p.x,
                -x.y * p.y,
                -x.y,
            ]),
        ];
        for row in rows {
            ata += row * row.transpose();
        }
    }
    let eigen = ata.symmetric_eigen();
    let h = eigen
        .eigenvectors
        .column(eigen.eigenvalues.imin())
        .into_owned();
    let hn = na::Matrix3::new(h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);
    let homography = ti.try_inverse()? * hn * tp;
    let (h1, h2, h3) = (
        homography.column(0),
        homography.column(1),
        homography.column(2),
    );
    let scale = 2.0 / (h1.norm() + h2.norm());
    let mut sign = 1.0;
    if (h3 * scale).z < 0.0 {
        sign = -1.0;
    }
    let (r1, r2, t) = (h1 * scale * sign, h2 * scale * sign, h3 * scale * sign);
    let rotation = na::Matrix3::from_columns(&[r1, r2, r1.cross(&r2)]);
    // The nearest rotation.
    let svd = rotation.svd(true, true);
    let (u, v_t) = (svd.u?, svd.v_t?);
    let mut nearest = u * v_t;
    if nearest.determinant() < 0.0 {
        nearest = u * na::Matrix3::from_diagonal(&na::Vector3::new(1.0, 1.0, -1.0)) * v_t;
    }
    let rotation = na::UnitQuaternion::from_matrix(&nearest);
    Some(na::Isometry3::from_parts(
        na::Translation3::from(t.into_owned()),
        rotation,
    ))
}

/// Hartley normalisation: centred, mean distance √2. Returns the transform
/// and the moved points.
fn normalise(points: &[na::Point2<f64>]) -> Option<(na::Matrix3<f64>, Vec<na::Point2<f64>>)> {
    let n = points.len() as f64;
    let centre = points.iter().map(|p| p.coords).sum::<na::Vector2<f64>>() / n;
    let mean = points
        .iter()
        .map(|p| (p.coords - centre).norm())
        .sum::<f64>()
        / n;
    if mean <= 1e-12 {
        return None;
    }
    let s = std::f64::consts::SQRT_2 / mean;
    let transform = na::Matrix3::new(s, 0.0, -s * centre.x, 0.0, s, -s * centre.y, 0.0, 0.0, 1.0);
    let moved = points
        .iter()
        .map(|p| na::Point2::from((p.coords - centre) * s))
        .collect();
    Some((transform, moved))
}

/// Levenberg–Marquardt on the reprojection error, from `pose`; returns the
/// refined pose and its RMS error (px).
fn refine_pose(
    mut pose: na::Isometry3<f64>,
    pixels: &[[f64; 2]],
    object: &[[f64; 3]],
    camera: &Pinhole,
) -> (na::Isometry3<f64>, f64) {
    let residuals = |pose: &na::Isometry3<f64>| -> Option<na::DVector<f64>> {
        let mut r = na::DVector::zeros(2 * pixels.len());
        for (k, (pixel, point)) in pixels.iter().zip(object).enumerate() {
            let seen = camera.project(&(pose * na::Point3::from(*point)))?;
            r[2 * k] = seen.x - pixel[0];
            r[2 * k + 1] = seen.y - pixel[1];
        }
        Some(r)
    };
    let nudge = |pose: &na::Isometry3<f64>, delta: &na::Vector6<f64>| {
        let rotation = na::UnitQuaternion::from_scaled_axis(delta.fixed_rows::<3>(0).into_owned());
        na::Isometry3::from_parts(
            na::Translation3::from(pose.translation.vector + delta.fixed_rows::<3>(3)),
            rotation * pose.rotation,
        )
    };
    let Some(mut r) = residuals(&pose) else {
        return (pose, f64::INFINITY);
    };
    let mut cost = r.norm_squared();
    let mut damping = 1e-3;
    for _ in 0..50 {
        // A forward-difference Jacobian.
        let mut jacobian = na::DMatrix::zeros(r.len(), 6);
        for c in 0..6 {
            let mut delta = na::Vector6::zeros();
            delta[c] = 1e-7;
            let Some(moved) = residuals(&nudge(&pose, &delta)) else {
                return (pose, f64::INFINITY);
            };
            jacobian.set_column(c, &((moved - &r) / 1e-7));
        }
        let jtj = jacobian.transpose() * &jacobian;
        let jtr = jacobian.transpose() * &r;
        let mut improved = false;
        for _ in 0..10 {
            let mut system = jtj.clone();
            for d in 0..6 {
                system[(d, d)] += damping * jtj[(d, d)].max(1e-12);
            }
            let Some(step) = system.lu().solve(&(-&jtr)) else {
                break;
            };
            let step = na::Vector6::from_iterator(step.iter().copied());
            let candidate = nudge(&pose, &step);
            if let Some(next) = residuals(&candidate)
                && next.norm_squared() < cost
            {
                pose = candidate;
                r = next;
                let previous = cost;
                cost = r.norm_squared();
                damping = (damping / 3.0).max(1e-12);
                improved = previous - cost > 1e-12 * previous.max(1e-12);
                break;
            }
            damping *= 5.0;
        }
        if !improved {
            break;
        }
    }
    (pose, (cost / pixels.len() as f64).sqrt())
}

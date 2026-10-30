//! Vertex positions and triangles from the mesh files URDF descriptions
//! commonly name.

use std::path::Path;

/// Vertices, and triangles as indices into them.
pub(crate) type Triangles = (Vec<[f64; 3]>, Vec<[u32; 3]>);

/// Every vertex of an OBJ or STL file, by extension, in file units.
pub(crate) fn vertices(path: &Path) -> Result<Vec<[f64; 3]>, String> {
    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .map(str::to_ascii_lowercase);
    let bytes = std::fs::read(path).map_err(|error| error.to_string())?;
    match extension.as_deref() {
        Some("obj") => obj(&String::from_utf8_lossy(&bytes)),
        Some("stl") => stl(&bytes),
        _ => Err("only OBJ and STL meshes are read".into()),
    }
}

/// The vertices and triangles of an OBJ or STL file, by extension, in file
/// units. OBJ polygons are split into fans; STL triangles keep their own
/// three vertices each.
pub(crate) fn triangles(path: &Path) -> Result<Triangles, String> {
    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .map(str::to_ascii_lowercase);
    let bytes = std::fs::read(path).map_err(|error| error.to_string())?;
    match extension.as_deref() {
        Some("obj") => obj_faces(&String::from_utf8_lossy(&bytes)),
        Some("stl") => {
            let points = stl(&bytes)?;
            let count = u32::try_from(points.len() / 3).map_err(|_| "too many triangles")?;
            Ok((
                points,
                (0..count).map(|t| [3 * t, 3 * t + 1, 3 * t + 2]).collect(),
            ))
        }
        _ => Err("only OBJ and STL meshes are read".into()),
    }
}

/// `v` and `f` lines. A face corner is `i`, `i/t`, `i//n` or `i/t/n`, with
/// negative `i` counting back from the latest vertex.
fn obj_faces(text: &str) -> Result<Triangles, String> {
    let (mut points, mut faces) = (Vec::new(), Vec::new());
    for (number, line) in text.lines().enumerate() {
        let mut fields = line.split_whitespace();
        match fields.next() {
            Some("v") => points.push(coordinates(fields, number)?),
            Some("f") => {
                let corners = fields
                    .map(|corner| {
                        let index = corner.split('/').next().unwrap_or_default();
                        let index: i64 = index
                            .parse()
                            .map_err(|error| format!("line {}: {error}", number + 1))?;
                        let resolved = if index < 0 {
                            points.len() as i64 + index
                        } else {
                            index - 1
                        };
                        u32::try_from(resolved)
                            .ok()
                            .filter(|i| (*i as usize) < points.len())
                            .ok_or_else(|| format!("line {}: no vertex {index}", number + 1))
                    })
                    .collect::<Result<Vec<u32>, String>>()?;
                if corners.len() < 3 {
                    return Err(format!("line {}: a face needs three corners", number + 1));
                }
                faces.extend(
                    (1..corners.len() - 1).map(|k| [corners[0], corners[k], corners[k + 1]]),
                );
            }
            _ => {}
        }
    }
    Ok((points, faces))
}

/// `v x y z [w]` lines; faces, normals and texture coordinates are not needed
/// for a hull.
fn obj(text: &str) -> Result<Vec<[f64; 3]>, String> {
    let mut points = Vec::new();
    for (number, line) in text.lines().enumerate() {
        let mut fields = line.split_whitespace();
        if fields.next() == Some("v") {
            points.push(coordinates(fields, number)?);
        }
    }
    Ok(points)
}

fn stl(bytes: &[u8]) -> Result<Vec<[f64; 3]>, String> {
    // Binary: an 80-byte header, the triangle count, then 50 bytes per
    // triangle (normal, three vertices, attribute). ASCII files may also
    // begin with `solid`, so the length decides.
    if let Some(count) = bytes.get(80..84) {
        let count = u32::from_le_bytes(count.try_into().expect("four bytes")) as usize;
        if bytes.len() == 84 + 50 * count {
            let mut points = Vec::with_capacity(3 * count);
            for triangle in bytes[84..].as_chunks::<50>().0 {
                for vertex in triangle[12..48].as_chunks::<12>().0 {
                    let [x, y, z] = vertex.as_chunks::<4>().0 else {
                        unreachable!("twelve bytes are three floats")
                    };
                    points.push([x, y, z].map(|value| f32::from_le_bytes(*value) as f64));
                }
            }
            return Ok(points);
        }
    }
    let text = String::from_utf8_lossy(bytes);
    if !text.trim_start().starts_with("solid") {
        return Err("neither binary nor ASCII STL".into());
    }
    let mut points = Vec::new();
    for (number, line) in text.lines().enumerate() {
        let mut fields = line.split_whitespace();
        if fields.next() == Some("vertex") {
            points.push(coordinates(fields, number)?);
        }
    }
    Ok(points)
}

fn coordinates<'a>(
    mut fields: impl Iterator<Item = &'a str>,
    line: usize,
) -> Result<[f64; 3], String> {
    let mut point = [0.0; 3];
    for value in &mut point {
        let field = fields
            .next()
            .ok_or_else(|| format!("line {}: a vertex needs three coordinates", line + 1))?;
        *value = field
            .parse()
            .map_err(|error| format!("line {}: {error}", line + 1))?;
    }
    Ok(point)
}

//! Vertex positions from the mesh files URDF descriptions commonly name.

use std::path::Path;

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

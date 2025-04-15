#[cfg(not(feature = "to_py"))]
#[derive(Debug, Clone)]
pub struct LoadState {
    pub m: f64,
    pub x: [f64; 3],
    pub i: [f64; 9],
}

#[cfg(feature = "to_py")]
use pyo3::pyclass;

#[cfg(feature = "to_py")]
#[derive(Debug, Clone)]
#[pyclass]
pub struct LoadState {
    #[pyo3(get, set)]
    pub m: f64,
    #[pyo3(get, set)]
    pub x: [f64; 3],
    #[pyo3(get, set)]
    pub i: [f64; 9],
}

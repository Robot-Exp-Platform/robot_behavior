#[cfg(not(feature = "to_py"))]
#[derive(Debug, Clone)]
pub struct LoadState {
    pub m: f64,
    pub x: [f64; 3],
    pub i: [f64; 9],
}

#[cfg(feature = "to_py")]
use pyo3::{pyclass, pymethods};

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

#[cfg(feature = "to_py")]
mod to_py {
    use super::*;

    #[pymethods]
    impl LoadState {
        #[new]
        pub fn new(m: f64, x: [f64; 3], i: [f64; 9]) -> Self {
            LoadState { m, x, i }
        }

        pub fn __repr__(&self) -> String {
            format!(
                "LoadState(m={}, x=[{}, {}, {}], i=[{}, {}, {}, {}, {}, {}, {}, {}, {}])",
                self.m,
                self.x[0],
                self.x[1],
                self.x[2],
                self.i[0],
                self.i[1],
                self.i[2],
                self.i[3],
                self.i[4],
                self.i[5],
                self.i[6],
                self.i[7],
                self.i[8]
            )
        }
    }
}

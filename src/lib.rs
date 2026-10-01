pub mod declaration;
pub mod bridge;

use pyo3::prelude::*;

#[pymodule]
fn abr_fr3(m: &Bound<'_, PyModule>) -> PyResult<()> {
    bridge::register(m)
}

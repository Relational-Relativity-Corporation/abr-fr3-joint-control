// PyO3 bridge — exposes one relational step to Python (Blender).
// Metatron Dynamics, Inc. — relationalrelativity.dev
// V0.2.2 Verifier PASS — V-1 and V-2 closed.
//
// The bridge introduces no operator, no parameter, no topology.
// Everything is declared in declaration.rs and passed through unchanged.
// The canonical abr_kernel operators execute. The bridge is a display window
// into the operator chain — not a reimplementation of it.
//
// Phase 1 (prior_e_spatial = None): spatial kernel only — operator_e.
//   Command = M⁻¹(E_spatial.spatial[0][EDGE_TAU_TO_THETA])
//
// Phase 2 (prior_e_spatial = Some(...)): full kernel — operator_e_v5.
//   Command = M⁻¹(E_v5.spatial[0][EDGE_TAU_TO_THETA])
//
// In both phases the command is driven directly by the established kernel
// output at the causal terminal edge. No bridge-invented combination used.
//
// Rest state: called with all zeros — field resolves to zero throughout.
// A=0, B=0, R=0, joint_delta=0. Declared ground before motion begins.

use pyo3::prelude::*;
use abr_kernel::operators::{
    operator_a, operator_b, compute_rho, operator_r,
    operator_e, operator_e_v5,
    EdgeField, PersistenceState,
};
use crate::declaration::{
    declared_topology, build_node_field,
    m_inv_tau, RHO_BASE, CHI0_TAU,
    EDGE_TAU_TO_THETA,
};

// ── StepResult ────────────────────────────────────────────────────────────────

#[pyclass]
#[derive(Clone, Debug)]
pub struct StepResult {
    /// A values — signed directed contrast at each edge [edge0, edge1, edge2]
    #[pyo3(get)]
    pub a: Vec<f64>,

    /// B values — accumulated relational state at each edge
    #[pyo3(get)]
    pub b: Vec<f64>,

    /// R values — spatial R at each edge (for display trace)
    #[pyo3(get)]
    pub r: Vec<f64>,

    /// Joint torque command in N·m — M⁻¹(E[EDGE_TAU_TO_THETA])
    /// Driven directly by established kernel output at causal terminal edge.
    #[pyo3(get)]
    pub joint_delta: f64,

    /// E_spatial output — pass as prior_e_spatial on the next step
    /// to activate Phase 2 persistence evaluation.
    #[pyo3(get)]
    pub e_spatial: Vec<Vec<f64>>,
}

#[pymethods]
impl StepResult {
    fn __repr__(&self) -> String {
        format!(
            "StepResult(a={:?}, b={:?}, r={:?}, joint_delta={:.4})",
            self.a, self.b, self.r, self.joint_delta
        )
    }
}

// ── advance_relational_step ───────────────────────────────────────────────────

#[pyfunction]
#[pyo3(signature = (m_dp_prior, m_dp, m_dtau, m_dtheta, prior_e_spatial=None))]
pub fn advance_relational_step(
    m_dp_prior: f64,
    m_dp: f64,
    m_dtau: f64,
    m_dtheta: f64,
    prior_e_spatial: Option<Vec<Vec<f64>>>,
) -> PyResult<StepResult> {
    let rel     = declared_topology();
    let field   = build_node_field(m_dp_prior, m_dp, m_dtau, m_dtheta);
    let pairs: &[(usize, usize)] = &[];
    let cc: &[f64]               = &[];

    // Spatial A, B, R — computed for display trace in all cases
    let a   = operator_a(&field, &rel, pairs);
    let rho = compute_rho(&a, &rel, RHO_BASE, CHI0_TAU);
    let b   = operator_b(&a, &rel);
    let r   = operator_r(&b, &rel, &rho, cc);

    let a_vals: Vec<f64> = a.spatial[0].clone();
    let b_vals: Vec<f64> = b.spatial[0].clone();
    let r_vals: Vec<f64> = r.spatial[0].clone();

    let (e_out, joint_delta) = match prior_e_spatial {
        None => {
            // Phase 1 — first declared observation. Spatial kernel only.
            // Kernel V8: at the first declared observation, only the spatial
            // kernel is evaluated.
            let e = operator_e(&field, &rel, pairs, cc, RHO_BASE, CHI0_TAU);
            let cmd = m_inv_tau(e.spatial[0][EDGE_TAU_TO_THETA]);
            (e.spatial, cmd)
        }
        Some(prior_spatial) => {
            // Phase 2 — prior declared observation exists. Full kernel.
            // Kernel V8: PersistenceState::from_prior requires actual
            // observation output — not a model or static configuration.
            let prior_ef = EdgeField {
                spatial:    prior_spatial,
                comp:       vec![],
                comp_pairs: vec![],
                k:          1,
            };
            let prior_state = PersistenceState::from_prior(prior_ef);
            let (e, _persistence) = operator_e_v5(
                &field, &rel, pairs, cc, &prior_state, RHO_BASE, CHI0_TAU,
            );
            // _persistence (A_p, B_p, R_p) available for future declared use.
            // Not combined with E_v5 here — no such combination established
            // by Kernel V8. Return to Origin if persistence output should
            // drive the actuator or enter the field.
            let cmd = m_inv_tau(e.spatial[0][EDGE_TAU_TO_THETA]);
            (e.spatial, cmd)
        }
    };

    Ok(StepResult {
        a: a_vals,
        b: b_vals,
        r: r_vals,
        joint_delta,
        e_spatial: e_out,
    })
}

// ── PyO3 0.22 module registration ─────────────────────────────────────────────

#[pymodule]
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<StepResult>()?;
    m.add_function(wrap_pyfunction!(advance_relational_step, m)?)?;
    Ok(())
}

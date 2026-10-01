// PyO3 bridge — exposes one relational step to Python (Blender).
// Metatron Dynamics, Inc. — relationalrelativity.dev
//
// The bridge introduces no operator, no parameter, no topology.
// Everything is declared in declaration.rs and passed through unchanged.
// The canonical abr_kernel operators execute. The bridge is a display window
// into the operator chain — not a reimplementation of it.
//
// Python call (from Blender):
//   import abr_fr3
//   result = abr_fr3.advance_relational_step(
//       m_dp_prior, m_dp, m_dtau, m_dtheta,
//       prior_e_spatial=None   # None on first step; pass result.e_spatial after
//   )
//   joint_delta = result.joint_delta   # physical torque command in N·m

use pyo3::prelude::*;
use abr_kernel::operators::{
    operator_e, operator_e_v5,
    EdgeField, PersistenceState,
};
use crate::declaration::{
    declared_topology, build_node_field,
    m_inv_tau, RHO_BASE, CHI0_TAU,
    EDGE_TAU_TO_THETA,
};

// ── StepResult — returned to Python per relational step ──────────────────────

#[pyclass]
#[derive(Clone, Debug)]
pub struct StepResult {
    /// A values — signed directed contrast at each edge [edge0, edge1, edge2]
    #[pyo3(get)]
    pub a: Vec<f64>,

    /// B values — accumulated relational state at each edge
    #[pyo3(get)]
    pub b: Vec<f64>,

    /// R values — resolved field at each edge (spatial + persistence if Phase 2)
    #[pyo3(get)]
    pub r: Vec<f64>,

    /// Joint torque command in N·m — M⁻¹(R[edge2])
    /// Edge 2 (Δτ → Δθ) is the causal terminal edge — what drives the joint.
    #[pyo3(get)]
    pub joint_delta: f64,

    /// Spatial EdgeField output — pass as prior_e_spatial on the next step
    /// to activate Phase 2 (persistence). Serialized as Vec<Vec<f64>>.
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

/// Advance the declared FR3 joint 1 relational field by one step.
///
/// All inputs are dimensionless — already projected through M by the caller.
/// Use declaration::m_tau(), m_p(), m_theta() to orient physical values.
///
/// Args:
///   m_dp_prior       — M(ΔP) from the prior step (dimensionless)
///   m_dp             — M(ΔP) current step (dimensionless)
///   m_dtau           — M(Δτ) current step (dimensionless)
///   m_dtheta         — M(Δθ) current step (dimensionless)
///   prior_e_spatial  — None on first step; pass result.e_spatial from prior
///                      step to activate Phase 2 persistence evaluation
///
/// Returns: StepResult with A, B, R per edge and joint_delta in N·m
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
    let pairs: &[(usize, usize)] = &[]; // no component pairs declared
    let cc: &[f64]               = &[]; // no cross-topology coupling declared

    let (e_spatial, a_vals, b_vals, r_vals) = match prior_e_spatial {
        None => {
            // Phase 1 only — first declared observation
            let e = operator_e(&field, &rel, pairs, cc, RHO_BASE, CHI0_TAU);
            // Reconstruct A and B for the trace — re-run to expose intermediates
            use abr_kernel::operators::{operator_a, operator_b, compute_rho, operator_r};
            let a = operator_a(&field, &rel, pairs);
            let rho = compute_rho(&a, &rel, RHO_BASE, CHI0_TAU);
            let b = operator_b(&a, &rel);
            let r = operator_r(&b, &rel, &rho, cc);
            let a_v = a.spatial[0].clone();
            let b_v = b.spatial[0].clone();
            let r_v = r.spatial[0].clone();
            (e, a_v, b_v, r_v)
        }
        Some(prior_spatial) => {
            // Phase 2 — prior step exists, activate persistence
            let prior_ef = EdgeField {
                spatial:     prior_spatial,
                comp:        vec![],
                comp_pairs:  vec![],
                k:           1,
            };
            let prior_state = PersistenceState::from_prior(prior_ef);
            let (e, persistence) = operator_e_v5(
                &field, &rel, pairs, cc, &prior_state, RHO_BASE, CHI0_TAU
            );
            // Spatial A and B for trace
            use abr_kernel::operators::{operator_a, operator_b, compute_rho, operator_r};
            let a = operator_a(&field, &rel, pairs);
            let rho = compute_rho(&a, &rel, RHO_BASE, CHI0_TAU);
            let b = operator_b(&a, &rel);
            let r = operator_r(&b, &rel, &rho, cc);
            // R_final = R_spatial + R_persistence (combined)
            let r_combined: Vec<f64> = r.spatial[0].iter().enumerate()
                .map(|(i, &rv)| rv + persistence.r_persistence[0][i])
                .collect();
            let a_v = a.spatial[0].clone();
            let b_v = b.spatial[0].clone();
            (e, a_v, b_v, r_combined)
        }
    };

    // Command: R at edge 2 (Δτ → Δθ) — causal terminal edge
    let r_edge2    = r_vals[EDGE_TAU_TO_THETA];
    let joint_delta = m_inv_tau(r_edge2);
    let e_out       = e_spatial.spatial.clone();

    Ok(StepResult {
        a: a_vals,
        b: b_vals,
        r: r_vals,
        joint_delta,
        e_spatial: e_out,
    })
}


// -- PyO3 0.22 module registration ---------------------------------------------

#[pymodule]
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<StepResult>()?;
    m.add_function(wrap_pyfunction!(advance_relational_step, m)?)?;
    Ok(())
}

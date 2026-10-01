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
    operator_a, operator_b, compute_rho, operator_r,
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

    /// R values — spatial R at each edge (for display trace)
    #[pyo3(get)]
    pub r: Vec<f64>,

    /// Joint torque command in N·m.
    /// Phase 1 (first step): M⁻¹(E_spatial[edge2])
    /// Phase 2 (subsequent): M⁻¹(E_v5[edge2])
    /// In both cases driven directly by the established kernel output,
    /// not by any bridge-constructed combination.
    #[pyo3(get)]
    pub joint_delta: f64,

    /// E_spatial output — pass as prior_e_spatial on the next step
    /// to activate Phase 2 (PersistenceState::from_prior).
    /// This is the kernel's EdgeField.spatial, carried forward as declared.
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
/// Phase 1 (prior_e_spatial = None): spatial kernel only — operator_e.
///   Kernel V8: first declared observation, only spatial kernel evaluated.
///   Command = M⁻¹(E_spatial.spatial[0][EDGE_TAU_TO_THETA])
///
/// Phase 2 (prior_e_spatial = Some(...)): full kernel — operator_e_v5.
///   Kernel V8: PersistenceState::from_prior(e_prior); A_p → B_p → R_p.
///   Command = M⁻¹(E_v5.spatial[0][EDGE_TAU_TO_THETA])
///
/// In both phases the command is driven directly by the established kernel
/// output at the causal terminal edge (EDGE_TAU_TO_THETA). No bridge-invented
/// combination of spatial and persistence outputs is used.
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

    // Spatial A, B, R — computed in all cases for the display trace.
    // These are the established kernel intermediates, exposed for visualization.
    // They are NOT used to drive the actuator — the established E output is.
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
            // kernel is evaluated. operator_e = R(B(A(x)), ρ(A(x))).
            let e = operator_e(&field, &rel, pairs, cc, RHO_BASE, CHI0_TAU);
            let cmd = m_inv_tau(e.spatial[0][EDGE_TAU_TO_THETA]);
            (e.spatial, cmd)
        }
        Some(prior_spatial) => {
            // Phase 2 — prior declared observation exists. Full kernel.
            // Kernel V8: PersistenceState::from_prior requires actual
            // observation output — not a model or static configuration.
            // operator_e_v5 evaluates Phase 1 then A_p → B_p → R_p.
            // Command is driven by E_v5.spatial[0][EDGE_TAU_TO_THETA] —
            // the established kernel output at the causal terminal edge.
            // No combination with persistence output is introduced here.
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
            // _persistence (A_p, B_p, R_p) is available for future declared
            // use. It is not combined with E_v5 here — no such combination
            // is established by Kernel V8. Return to Origin if persistence
            // output should drive the actuator or enter the field.
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

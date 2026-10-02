// M Declarations — FR3 Joint 1 Change-Based Relational Field
// Metatron Dynamics, Inc. — relationalrelativity.dev
// Origin declaration: 2026-10-01
// Bounded over D. No claim beyond D.
//
// Observable chain: ΔP → Δτ → Δθ (change-based, per relational step)
// One relational step = one declared change transition per animation frame.
// Physical provenance: FR3 URDF (Apache 2.0), FR3 Product Manual R02210/1.5
// Power ceiling: τ_max × ω_max = 87.0 × 2.0 = 174.0 W (OC-FR3-6 closed)
// Position range: ±2.3093 rad (from URDF joint limits, fr3_joint1)
//
// Rest state declaration: field ground at ΔP=0, Δτ=0, Δθ=0.
// All A=0, B=0, R=0. joint_delta=0. Arm at declared neutral.
// Equivalent to relative inertia = 0 at the joint — declared ground
// before any relational step advances. Analogous to G-code machine home.

use abr_kernel::operators::{DeclaredRelations, EdgeProvenance, NodeField};

// ── M-declared physical ceilings ─────────────────────────────────────────────
// Source: FR3 URDF (Relational-Relativity-Corporation/franka_description),
//         FR3 Product Manual R02210/1.5

/// Maximum joint torque at A1 — from URDF <limit effort="87.0">
pub const TAU_MAX: f64 = 87.0;

/// Maximum joint velocity at A1 — from URDF <limit velocity="2.0">
pub const OMEGA_MAX: f64 = 2.0;

/// Peak mechanical power — declared as τ_max × ω_max (OC-FR3-6 closed)
pub const P_MAX: f64 = TAU_MAX * OMEGA_MAX; // 174.0 W

/// Joint position range at A1 — from URDF <limit lower="-2.3093" upper="2.3093">
pub const THETA_MAX: f64 = 2.3093;

// ── M-declared operator parameters ───────────────────────────────────────────

/// ρ_base — neutral pose floor declaration.
/// Highest inertia configuration (I_eff ≈ 0.86 kg·m² at neutral pose).
/// Physical arm: ρ_base is configuration-dependent via FCI inertia matrix.
pub const RHO_BASE: f64 = 1.0;

/// χ₀ for torque channel — smallest meaningful relational contrast
/// in the dimensionless domain after M orientation.
pub const CHI0_TAU: f64 = 1.0 / TAU_MAX; // ≈ 0.01149

/// χ₀ for power channel
pub const CHI0_P: f64 = 1.0 / P_MAX; // ≈ 0.005747

// ── Node indices — declared topology ─────────────────────────────────────────
// Four nodes, three spatial edges.
// Edge 0: ΔP_prior → ΔP   (Coupling — prior power state constrains current)
// Edge 1: ΔP → Δτ         (Coupling — power change drives torque change)
// Edge 2: Δτ → Δθ         (Coupling — torque change drives position change)
//
// Interior locus: node 2 (Δτ) — target of edge 1, source of edge 2.
// Full-operator admissibility condition satisfied. ABR kernel applies.
// R is not optional.

pub const NODE_DP_PRIOR: usize = 0;
pub const NODE_DP:       usize = 1;
pub const NODE_DTAU:     usize = 2;
pub const NODE_DTHETA:   usize = 3;

pub const EDGE_PRIOR_TO_P:   usize = 0; // ΔP_prior → ΔP
pub const EDGE_P_TO_TAU:     usize = 1; // ΔP → Δτ
pub const EDGE_TAU_TO_THETA: usize = 2; // Δτ → Δθ

pub const N_NODES: usize = 4;

/// Build the declared relational topology.
/// Called once — the topology does not change across relational steps.
pub fn declared_topology() -> DeclaredRelations {
    DeclaredRelations::from_edges_with_provenance(
        N_NODES,
        vec![
            (NODE_DP_PRIOR, NODE_DP),
            (NODE_DP,       NODE_DTAU),
            (NODE_DTAU,     NODE_DTHETA),
        ],
        vec![
            EdgeProvenance::Coupling,
            EdgeProvenance::Coupling,
            EdgeProvenance::Coupling,
        ],
    )
}

// ── M orientation functions ───────────────────────────────────────────────────

/// M(Δτ): dimensionless torque change. [−87, +87] N·m → [−1, +1].
pub fn m_tau(delta_tau_nm: f64) -> Option<f64> {
    let v = delta_tau_nm / TAU_MAX;
    if v.abs() <= 1.0 { Some(v) } else { None }
}

/// M(ΔP): dimensionless power change. [−174, +174] W → [−1, +1].
pub fn m_p(delta_p_w: f64) -> Option<f64> {
    let v = delta_p_w / P_MAX;
    if v.abs() <= 1.0 { Some(v) } else { None }
}

/// M(Δθ): dimensionless position change. [−2.3093, +2.3093] rad → [−1, +1].
pub fn m_theta(delta_theta_rad: f64) -> Option<f64> {
    let v = delta_theta_rad / THETA_MAX;
    if v.abs() <= 1.0 { Some(v) } else { None }
}

/// M⁻¹(τ): map dimensionless R output back to physical torque command.
/// Clamped to declared actuator admission envelope ±87.0 N·m.
pub fn m_inv_tau(r_result: f64) -> f64 {
    (r_result * TAU_MAX).clamp(-TAU_MAX, TAU_MAX)
}

/// M reduction: derive ΔP from Δτ and ω.
/// P = τ × ω at the motor shaft — declared physical relationship.
pub fn m_p_from_delta_tau_omega(delta_tau_nm: f64, omega_rad_s: f64) -> f64 {
    delta_tau_nm * omega_rad_s
}

/// Build a NodeField from dimensionless M-oriented change values.
/// All values must already be projected through M before calling.
pub fn build_node_field(
    m_dp_prior: f64,
    m_dp: f64,
    m_dtau: f64,
    m_dtheta: f64,
) -> NodeField {
    NodeField::new(vec![vec![m_dp_prior, m_dp, m_dtau, m_dtheta]])
}

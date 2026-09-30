// Conventional Treatment — Quasi-Static Torque Calculation
// Metatron Dynamics, Inc.
//
// This is the honest conventional answer. It is not handicapped.
// It uses identical declared inputs to the relational treatment.
// It is declared here so its parameterization is fully visible.
//
// τ_conventional[n] = F_load × L[n]
// where F_load and L[n] are the same declared quantities
// available to the relational treatment through M.

use crate::fr3_geometry::{L1_LEVER_ARM_M, L2_LEVER_ARM_M};

/// Conventional torque result — fully parameterized, nothing hidden.
#[derive(Debug, Clone)]
pub struct ConventionalResult {
    pub f_load_n: f64,
    pub l1_m: f64,
    pub tau_a1_nm: f64,
    pub l2_m: Option<f64>,
    pub tau_a2_nm: Option<f64>,
}

/// Compute conventional torque at A1 only (Experiment 1).
/// τ_A1 = F_load × L1
/// Parameters:
///   f_load_n: declared end-effector force (N) — shared with relational
///   L1:       declared lever arm constant at A1 — shared with relational
/// No tuned parameters. No fitted model. No learned weights.
pub fn conventional_exp1(f_load_n: f64) -> ConventionalResult {
    let tau_a1 = f_load_n * L1_LEVER_ARM_M;
    ConventionalResult {
        f_load_n,
        l1_m: L1_LEVER_ARM_M,
        tau_a1_nm: tau_a1,
        l2_m: None,
        tau_a2_nm: None,
    }
}

/// Compute conventional torque at A1 + A2 (Experiment 2).
/// τ_A1 = F_load × L1
/// τ_A2 = F_load × L2
/// Each joint requires its own declared moment arm.
/// The calculation is repeated per joint — no structural reuse.
pub fn conventional_exp2(f_load_n: f64) -> ConventionalResult {
    let tau_a1 = f_load_n * L1_LEVER_ARM_M;
    let tau_a2 = f_load_n * L2_LEVER_ARM_M;
    ConventionalResult {
        f_load_n,
        l1_m: L1_LEVER_ARM_M,
        tau_a1_nm: tau_a1,
        l2_m: Some(L2_LEVER_ARM_M),
        tau_a2_nm: Some(tau_a2),
    }
}

/// Physics ground truth — τ = F × L
/// Independent of both treatments. Same formula as conventional
/// for quasi-static single-joint. This is expected and declared.
/// Enters as comparison target only, never as input to either treatment.
pub fn physics_ground_truth_exp1(f_load_n: f64) -> f64 {
    f_load_n * L1_LEVER_ARM_M
}

pub fn physics_ground_truth_exp2(f_load_n: f64) -> (f64, f64) {
    (f_load_n * L1_LEVER_ARM_M, f_load_n * L2_LEVER_ARM_M)
}

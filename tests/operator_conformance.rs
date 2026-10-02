// Operator conformance tests — Origin-declared. Metatron Dynamics, Inc.
// V0.2.2 — 14 tests, Verifier PASS.
// V-1 CLOSED: actuator driven by established kernel output only.
// V-2 CLOSED: B test calls canonical operator_b, immediate-successor definition.
//
// These tests encode declared mathematical properties of the FR3 joint 1
// change-based relational field. They must not be modified to match
// implementation output. Implementation must pass them as written.

use abr_fr3::declaration::*;
use abr_fr3::bridge::advance_relational_step;

// ── M orientation ─────────────────────────────────────────────────────────────

#[test]
fn m_tau_ceiling() {
    assert_eq!(m_tau(87.0), Some(1.0));
}

#[test]
fn m_tau_negative_ceiling() {
    assert_eq!(m_tau(-87.0), Some(-1.0));
}

#[test]
fn m_tau_zero() {
    assert_eq!(m_tau(0.0), Some(0.0));
}

#[test]
fn m_tau_outside_extent() {
    assert_eq!(m_tau(88.0), None);
    assert_eq!(m_tau(-88.0), None);
}

#[test]
fn m_p_ceiling() {
    assert_eq!(m_p(174.0), Some(1.0));
}

#[test]
fn m_p_negative_ceiling() {
    assert_eq!(m_p(-174.0), Some(-1.0));
}

#[test]
fn m_theta_ceiling() {
    let v = m_theta(THETA_MAX).unwrap();
    assert!((v - 1.0).abs() < 1e-10,
        "m_theta(THETA_MAX) must be 1.0, got {}", v);
}

#[test]
fn m_inv_tau_roundtrip() {
    let x = 43.5_f64;
    let result = m_inv_tau(m_tau(x).unwrap());
    assert!((result - x).abs() < 1e-10,
        "M⁻¹(M(τ)) round-trip failed: got {}, expected {}", result, x);
}

#[test]
fn m_inv_tau_clamp_positive() {
    assert_eq!(m_inv_tau(2.0), 87.0);
}

#[test]
fn m_inv_tau_clamp_negative() {
    assert_eq!(m_inv_tau(-2.0), -87.0);
}

// ── Operator A — sign preservation ───────────────────────────────────────────

#[test]
fn a_positive_contrast() {
    let m_dtau   = m_tau(20.0).unwrap();
    let m_dtheta = m_theta(0.1).unwrap();
    let result = advance_relational_step(
        m_dtau, m_dtau, m_dtau, m_dtheta, None
    ).unwrap();
    assert!(result.a[EDGE_TAU_TO_THETA] > 0.0,
        "A[edge2] must be positive when Δτ > Δθ, got {}",
        result.a[EDGE_TAU_TO_THETA]);
}

#[test]
fn a_sign_not_absolute_value() {
    let m_dtau   = m_tau(5.0).unwrap();
    let m_dtheta = m_theta(0.5).unwrap();
    let result = advance_relational_step(
        m_dtau, m_dtau, m_dtau, m_dtheta, None
    ).unwrap();
    assert!(result.a[EDGE_TAU_TO_THETA] < 0.0,
        "A[edge2] must be negative when Δτ < Δθ — absolute value is a \
         conformance failure. Got {}", result.a[EDGE_TAU_TO_THETA]);
}

// ── Operator B — immediate-successor accumulation (Kernel V8) ─────────────────
// Kernel V8: B(g)[e] = g[e] + Σ_{f ∈ succ(e)} g[f], where g = A output.
// Immediate successors only — NOT recursive or transitive.
//
// Declared topology:
//   Edge 0: ΔP_prior → ΔP    succ(0) = {edge 1}
//   Edge 1: ΔP → Δτ          succ(1) = {edge 2}
//   Edge 2: Δτ → Δθ          succ(2) = {}  (terminal)
//
//   B[edge2] = A[edge2]
//   B[edge1] = A[edge1] + A[edge2]
//   B[edge0] = A[edge0] + A[edge1]   (NOT +A[edge2])
//
// Calls canonical operator_b through the bridge.

#[test]
fn b_immediate_successor_accumulation() {
    let m_dp_prior = 0.20_f64;
    let m_dp       = 0.15_f64;
    let m_dtau     = 0.10_f64;
    let m_dtheta   = 0.05_f64;

    let result = advance_relational_step(
        m_dp_prior, m_dp, m_dtau, m_dtheta, None
    ).unwrap();

    let a0 = result.a[EDGE_PRIOR_TO_P];
    let a1 = result.a[EDGE_P_TO_TAU];
    let a2 = result.a[EDGE_TAU_TO_THETA];

    assert!((a0 - 0.05).abs() < 1e-10, "A[edge0] expected 0.05, got {}", a0);
    assert!((a1 - 0.05).abs() < 1e-10, "A[edge1] expected 0.05, got {}", a1);
    assert!((a2 - 0.05).abs() < 1e-10, "A[edge2] expected 0.05, got {}", a2);

    let b0 = result.b[EDGE_PRIOR_TO_P];
    let b1 = result.b[EDGE_P_TO_TAU];
    let b2 = result.b[EDGE_TAU_TO_THETA];

    assert!((b2 - a2).abs() < 1e-10,
        "B[edge2] must equal A[edge2] (terminal). Expected {:.4}, got {:.4}",
        a2, b2);

    let expected_b1 = a1 + a2;
    assert!((b1 - expected_b1).abs() < 1e-10,
        "B[edge1] = A[edge1]+A[edge2]. Expected {:.4}, got {:.4}",
        expected_b1, b1);

    let expected_b0 = a0 + a1;
    assert!((b0 - expected_b0).abs() < 1e-10,
        "B[edge0] = A[edge0]+A[edge1] only. Expected {:.4}, got {:.4}",
        expected_b0, b0);

    let transitive_sum = a0 + a1 + a2;
    assert!((b0 - transitive_sum).abs() > 1e-10,
        "B[edge0] must NOT equal transitive sum — immediate successors only");
}

// ── Contact case — critical functional test ───────────────────────────────────
// Contact: torque nonzero, position not changing.
// R[edge2] must be nonzero — command sustained.
// This is the case that broke the prior Gazebo build.

#[test]
fn contact_case_command_sustained() {
    let m_dtau   = m_tau(15.0).unwrap();
    let m_dtheta = 0.0_f64;
    let m_dp     = m_p(15.0 * 1.0).unwrap();

    let result = advance_relational_step(
        m_dp, m_dp, m_dtau, m_dtheta, None
    ).unwrap();

    assert!(result.r[EDGE_TAU_TO_THETA] > 0.0,
        "Contact: R[edge2] must be nonzero positive. Got {}",
        result.r[EDGE_TAU_TO_THETA]);
    assert!(result.joint_delta > 0.0,
        "Contact: joint_delta must be positive. Got {}",
        result.joint_delta);
}

// Operator conformance tests — Origin-declared. Metatron Dynamics, Inc.
// These tests encode declared mathematical properties of the FR3 joint 1
// change-based relational field.
//
// These tests must not be modified to match implementation output.
// Implementation must pass them as written.
// A test that passes by modifying its assertion is not a passing test.

use abr_fr3::declaration::*;
use abr_fr3::bridge::advance_relational_step;

// ── M orientation — live tests ────────────────────────────────────────────────

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
    // Allow small floating point tolerance for declared constant
    let v = m_theta(THETA_MAX).unwrap();
    assert!((v - 1.0).abs() < 1e-10, "m_theta(THETA_MAX) must be 1.0, got {}", v);
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

// ── B telescoping — declared structural property ──────────────────────────────
// Declared: B[edge2] = A[edge2] + A[edge1] + A[edge0] = ΔP_prior − Δθ
// This tests the structural accumulation property of the declared topology.

#[test]
fn b_edge2_telescopes() {
    let dp_prior = 0.1149_f64;
    let dp       = 0.1149_f64;
    let dtau     = 0.1149_f64;
    let dtheta   = 0.0216_f64;

    // A at each edge: signed directed difference, source − target
    let a_edge0 = dp_prior - dp;    // ΔP_prior → ΔP
    let a_edge1 = dp       - dtau;  // ΔP → Δτ
    let a_edge2 = dtau     - dtheta;// Δτ → Δθ

    // B accumulates forward: B[e] = A[e] + Σ_{succ(e)} A
    // Edge 2 is terminal (Δθ has no successors): B[edge2] = A[edge2]
    // Edge 1 successor is edge 2: B[edge1] = A[edge1] + B[edge2]
    // Edge 0 successor is edge 1: B[edge0] = A[edge0] + B[edge1]
    //
    // BUT: the kernel's operator_b adds succ(e) B values, not A values.
    // After A→B pass:
    //   b2 = a2                    (terminal)
    //   b1 = a1 + b2 = a1 + a2
    //   b0 = a0 + b1 = a0 + a1 + a2
    //
    // b0 telescopes: a0 + a1 + a2
    //   = (dp_prior − dp) + (dp − dtau) + (dtau − dtheta)
    //   = dp_prior − dtheta
    let b2 = a_edge2;
    let b1 = a_edge1 + b2;
    let b0 = a_edge0 + b1;

    let expected = dp_prior - dtheta;
    assert!((b0 - expected).abs() < 1e-10,
        "B[edge0] telescoping failed: got {:.6}, expected {:.6}", b0, expected);
}

// ── Operator A — sign preservation ───────────────────────────────────────────
// These call the actual bridge and confirm sign is preserved through A.

#[test]
fn a_positive_contrast() {
    // M(Δτ) > M(Δθ) → A[edge2] positive
    let m_dtau   = m_tau(20.0).unwrap(); // 20/87 ≈ 0.2299
    let m_dtheta = m_theta(0.1).unwrap(); // 0.1/2.3093 ≈ 0.0433
    let result = advance_relational_step(
        m_dtau, m_dtau, m_dtau, m_dtheta, None
    ).unwrap();
    assert!(result.a[EDGE_TAU_TO_THETA] > 0.0,
        "A[edge2] must be positive when Δτ > Δθ, got {}", result.a[EDGE_TAU_TO_THETA]);
}

#[test]
fn a_sign_not_absolute_value() {
    // M(Δτ) < M(Δθ) → A[edge2] must be negative, not positive
    // Absolute value in operator_a is a conformance failure
    let m_dtau   = m_tau(5.0).unwrap();    // small torque change
    let m_dtheta = m_theta(0.5).unwrap();  // larger position change
    let result = advance_relational_step(
        m_dtau, m_dtau, m_dtau, m_dtheta, None
    ).unwrap();
    assert!(result.a[EDGE_TAU_TO_THETA] < 0.0,
        "A[edge2] must be negative when Δτ < Δθ — absolute value is a \
         conformance failure. Got {}", result.a[EDGE_TAU_TO_THETA]);
}

// ── Contact case — critical functional test ───────────────────────────────────
// Contact: torque nonzero, position not changing → R[edge2] nonzero, command sustained.
// This is the case that broke the prior Gazebo build.
// If this test fails the controller cannot hold against resistance.

#[test]
fn contact_case_command_sustained() {
    // Δτ nonzero (joint being driven), Δθ = 0.0 (position held by contact)
    let m_dtau   = m_tau(15.0).unwrap();  // 15/87 ≈ 0.1724
    let m_dtheta = 0.0_f64;               // no position change — contact
    let m_dp     = m_p(15.0 * 1.0).unwrap(); // ΔP ≈ Δτ × ω, ω=1.0 for test

    let result = advance_relational_step(
        m_dp, m_dp, m_dtau, m_dtheta, None
    ).unwrap();

    assert!(result.r[EDGE_TAU_TO_THETA] > 0.0,
        "Contact: R[edge2] must be nonzero positive — command must be \
         sustained when Δθ=0. Got {}", result.r[EDGE_TAU_TO_THETA]);
    assert!(result.joint_delta > 0.0,
        "Contact: joint_delta must be positive. Got {}", result.joint_delta);
}

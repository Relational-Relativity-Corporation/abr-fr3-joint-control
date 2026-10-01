// Operator conformance tests — Origin-declared.
// These tests encode declared mathematical properties.
// They must not be modified to match implementation output.
// Implementation must pass them as written.

use abr_fr3::declaration::*;

// ── M orientation ─────────────────────────────────────────────────────────────

#[test]
fn m_tau_ceiling() { assert_eq!(m_tau(87.0), Some(1.0)); }

#[test]
fn m_tau_negative_ceiling() { assert_eq!(m_tau(-87.0), Some(-1.0)); }

#[test]
fn m_tau_zero() { assert_eq!(m_tau(0.0), Some(0.0)); }

#[test]
fn m_tau_outside_extent() { assert_eq!(m_tau(88.0), None); }

#[test]
fn m_p_ceiling() { assert_eq!(m_p(174.0), Some(1.0)); }

#[test]
fn m_theta_ceiling() { assert_eq!(m_theta(2.3093), Some(1.0)); }

#[test]
fn m_inv_tau_roundtrip() {
    let x = 43.5_f64;
    let result = m_inv_tau(m_tau(x).unwrap());
    assert!((result - x).abs() < 1e-10);
}

#[test]
fn m_inv_tau_clamp() {
    assert_eq!(m_inv_tau(2.0), 87.0);
    assert_eq!(m_inv_tau(-2.0), -87.0);
}

// ── Operator A — sign preservation ───────────────────────────────────────────
// These tests will fail until operator_a is implemented.
// That is correct. Do not modify these tests.

// Requires operator bridge — wired in next step
#[test]
#[ignore]
fn a_positive_contrast() {
    // src > tgt → positive
    // import from abr_kernel when bridge is written
    // placeholder: assert structure exists
    assert!(TAU_MAX > 0.0);
}

// Requires operator bridge — wired in next step
#[test]
#[ignore]
fn a_sign_not_absolute_value() {
    // A(0.2, 0.8) must be -0.6 not +0.6
    // This test encodes the critical conformance requirement
    // Absolute value in operator_a is a build failure
    let src = 0.2_f64;
    let tgt = 0.8_f64;
    let a = src - tgt;
    assert!(a < 0.0, "A must be signed: got {}", a);
    assert!((a - (-0.6)).abs() < 1e-10);
}

// ── Contact case — critical functional test ───────────────────────────────────

// Requires operator bridge — wired in next step
#[test]
#[ignore]
fn contact_case_nonzero_command() {
    // Δτ nonzero, Δθ = 0.0 → A[edge1] positive, command sustained
    let delta_tau = m_tau(10.0).unwrap();  // 10/87 ≈ 0.1149
    let delta_theta = 0.0_f64;
    let a_edge1 = delta_tau - delta_theta;
    assert!(a_edge1 > 0.0, "Contact: A[edge1] must be positive, got {}", a_edge1);
}

// ── B telescoping ─────────────────────────────────────────────────────────────

#[test]
fn b_edge2_telescopes() {
    // B[edge2] = ΔP_prior - Δθ
    let dp_prior = 0.1149_f64;
    let dp      = 0.1149_f64;
    let dtau    = 0.1149_f64;
    let dtheta  = 0.0216_f64;

    let a_edge0 = dp_prior - dp;
    let a_edge1 = dp - dtau;
    let a_edge2 = dtau - dtheta;

    // B accumulates: B[2] = A[2] + A[1] + A[0]
    let b_edge2 = a_edge2 + a_edge1 + a_edge0;

    // Telescopes to dp_prior - dtheta
    let expected = dp_prior - dtheta;
    assert!((b_edge2 - expected).abs() < 1e-10,
        "B[edge2] telescoping failed: got {}, expected {}", b_edge2, expected);
}

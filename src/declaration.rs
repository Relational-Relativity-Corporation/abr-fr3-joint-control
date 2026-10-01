// M declarations — FR3 regional orientation.
// Metatron Dynamics, Inc. — relationalrelativity.dev

// ── Declared ceilings ─────────────────────────────────────────────────────────

pub const TAU_MAX: f64 = 87.0;
pub const P_MAX: f64 = 174.0;
pub const THETA_MAX: f64 = 2.3093;

// ── Operator parameters ───────────────────────────────────────────────────────

pub const RHO_BASE: f64 = 1.0;
pub const CHI0_TAU: f64 = 1.0 / 87.0;
pub const CHI0_P: f64 = 1.0 / 174.0;

// ── M orientation ─────────────────────────────────────────────────────────────

/// Orient a value against a declared ceiling.
/// Values outside declared extent are not admitted — return None.
fn orient(observed: f64, ceiling: f64) -> Option<f64> {
    let oriented = observed / ceiling;
    if oriented.abs() > 1.0 {
        None
    } else {
        Some(oriented)
    }
}

/// M_tau — torque [N·m] → [−1, 1].
pub fn m_tau(tau: f64) -> Option<f64> {
    orient(tau, TAU_MAX)
}

/// M_p — power [W] → [−1, 1].
pub fn m_p(p: f64) -> Option<f64> {
    orient(p, P_MAX)
}

/// M_theta — angle [rad] → [−1, 1].
pub fn m_theta(theta: f64) -> Option<f64> {
    orient(theta, THETA_MAX)
}

/// Reverse M — R dimensionless result → torque command [N·m],
/// clamped to declared actuator envelope ±87.0 N·m.
pub fn m_inv_tau(r: f64) -> f64 {
    (r * TAU_MAX).clamp(-TAU_MAX, TAU_MAX)
}

/// Declared M reduction: P = τ·ω.
pub fn m_p_from_tau_omega(tau: f64, omega: f64) -> f64 {
    tau * omega
}

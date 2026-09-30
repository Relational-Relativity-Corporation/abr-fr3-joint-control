// FR3 Geometry — Declared Constants
// All values from published Franka Research 3 sources.
// No values are computed here. No legacy math here.
// Source 1: Franka Research 3 Datasheet R02212 2.4
//   https://franka.de/hubfs/Datasheet_FrankaResearch3_R02212_2.4.pdf
// Source 2: frankarobotics/franka_description (Apache 2.0)
//   https://github.com/frankarobotics/franka_description

/// FR3 maximum reach — end-effector to base, fully extended.
/// Source: Datasheet R02212 2.4, "Maximum reach: 855 mm"
pub const FR3_MAX_REACH_M: f64 = 0.855;

/// FR3 rated payload — maximum end-effector load.
/// Source: Datasheet R02212 2.4, "Rated payload: 3 kg"
pub const FR3_RATED_PAYLOAD_KG: f64 = 3.0;

/// FR3 joint torque limits — A1 through A4.
/// Source: Datasheet R02212 2.4, "A1-A4: ±87 Nm"
/// These are declared admissibility boundaries in D.
pub const FR3_TORQUE_LIMIT_A1_A4_NM: f64 = 87.0;

/// FR3 joint torque limits — A5 through A7.
/// Source: Datasheet R02212 2.4, "A5-A7: ±12 Nm"
pub const FR3_TORQUE_LIMIT_A5_A7_NM: f64 = 12.0;

/// FR3 joint position limits — A1 (radians).
/// Source: Datasheet R02212 2.4, "A1: -166/166 deg"
pub const FR3_A1_LIMIT_MIN_RAD: f64 = -2.8973; // -166 deg
pub const FR3_A1_LIMIT_MAX_RAD: f64 = 2.8973;  // +166 deg

/// FR3 joint position limits — A2 (radians).
/// Source: Datasheet R02212 2.4, "A2: -105/105 deg"
pub const FR3_A2_LIMIT_MIN_RAD: f64 = -1.8326; // -105 deg
pub const FR3_A2_LIMIT_MAX_RAD: f64 = 1.8326;  // +105 deg

/// Declared configuration for experiments.
/// θ₁ = 0.0 rad (A1 neutral — arm pointing forward)
/// θ₂ = 0.0 rad (A2 neutral — arm horizontal)
/// Fixed within D. Configuration variation is outside D.
pub const DECLARED_THETA_1_RAD: f64 = 0.0;
pub const DECLARED_THETA_2_RAD: f64 = 0.0;

/// Declared lever arm at A1 — at declared configuration.
/// This is a geometric fact at θ₁=0, θ₂=0: effective moment arm
/// equals the horizontal reach from A1 axis to end-effector.
/// At neutral configuration, this equals FR3_MAX_REACH_M.
/// Source: Datasheet R02212 2.4, reach 0.855 m.
/// NOT computed by M or operators. Declared constant.
pub const L1_LEVER_ARM_M: f64 = 0.855;

/// Declared lever arm at A2 — at declared configuration.
/// Effective moment arm from A2 axis to end-effector at θ₁=0, θ₂=0.
/// Declared working value: distance from shoulder joint A2 to
/// end-effector along the arm at neutral configuration.
/// Source: franka_description URDF link geometry.
/// OC-FR3-1: exact value pending URDF extraction.
pub const L2_LEVER_ARM_M: f64 = 0.316; // working value, OC-FR3-1

/// Standard gravity — declared physical constant.
pub const G_M_S2: f64 = 9.81;

/// Check that a torque value is within declared FR3 admissibility
/// bounds for joints A1-A4.
pub fn is_admitted_a1_a4(torque_nm: f64) -> bool {
    torque_nm.abs() <= FR3_TORQUE_LIMIT_A1_A4_NM
}

/// Check configuration is within declared FR3 joint limits.
pub fn theta1_in_limits(theta_rad: f64) -> bool {
    theta_rad >= FR3_A1_LIMIT_MIN_RAD && theta_rad <= FR3_A1_LIMIT_MAX_RAD
}

pub fn theta2_in_limits(theta_rad: f64) -> bool {
    theta_rad >= FR3_A2_LIMIT_MIN_RAD && theta_rad <= FR3_A2_LIMIT_MAX_RAD
}

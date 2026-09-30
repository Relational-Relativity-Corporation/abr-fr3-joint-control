// M Declaration — ABR Relational Topology for FR3 Joint Control
// Origin: Metatron Dynamics, Inc.
// Bounded over D. No claim beyond D.
//
// D: FR3 serial chain, quasi-static equilibrium, single-axis revolute
// joints A1 and A2, declared external load at end-effector,
// declared configuration θ₁=0.0 rad, θ₂=0.0 rad.
//
// M maps observable quantities from FR3 sensor interfaces into D.
// M declares geometric constants at the declared configuration.
// M does NOT compute the conventional torque calculation.
// M does NOT perform forward kinematics.
// M does NOT compute lever arms from joint angles.

use crate::operators::{DeclaredEdge, DeclaredRelations, EdgeKind, NodeField, PersistenceState};
use crate::fr3_geometry::*;

// ── Operator parameters ───────────────────────────────────────────────────────

/// ρ_base — declared working value, carried from abr_sensorimotor_interface.
/// OC-FR3-2: not yet characterized for this domain.
pub const RHO_BASE: f64 = 0.8;

/// χ₀ — declared in N·m, matching NodeField dimension.
/// Declared lower bound of meaningful torque distinction in D.
/// OC-FR3-3: characterization from FR3 sensor specification pending.
pub const CHI_0_NM: f64 = 1.0;

// ── Node labels ───────────────────────────────────────────────────────────────

pub const NODE_LOAD: &str = "load";
pub const NODE_TAU_MEAS_1: &str = "tau_measured_1";
pub const NODE_TAU_CMD_1: &str = "torque_cmd_1";
pub const NODE_TAU_MEAS_2: &str = "tau_measured_2";
pub const NODE_TAU_CMD_2: &str = "torque_cmd_2";

// ── M mapping functions ───────────────────────────────────────────────────────

/// Map payload mass to end-effector force observable.
/// x[load] = m_payload × g
/// Source: FR3 rated payload 3.0 kg, g = 9.81 m/s²
/// M maps this directly — no computation of torque here.
pub fn m_load_from_mass(mass_kg: f64) -> f64 {
    mass_kg * G_M_S2
}

/// Map FR3 link-side torque sensor reading to declared observable.
/// x[tau_measured_n] = sensor_reading_nm
/// Direct FR3 sensor output. No computation.
/// In simulation: tau_measured = physics ground truth (F × L).
/// On hardware: tau_measured = FR3 link-side torque sensor at joint n.
pub fn m_tau_measured(sensor_reading_nm: f64) -> f64 {
    sensor_reading_nm
}

// ── Experiment 1: A1 only ─────────────────────────────────────────────────────

/// Build NodeField for Experiment 1 — A1 only.
/// Observable inputs: load (N), tau_measured_1 (N·m).
/// Output node: torque_cmd_1 initialised to 0.0.
pub fn build_nodefield_exp1(load_n: f64, tau_meas_1_nm: f64) -> NodeField {
    NodeField::new(
        vec![
            NODE_LOAD.to_string(),
            NODE_TAU_MEAS_1.to_string(),
            NODE_TAU_CMD_1.to_string(),
        ],
        vec![load_n, tau_meas_1_nm, 0.0],
    )
}

/// Build declared relational topology for Experiment 1.
///
/// E-01: load → tau_measured_1 [Coupling]
///   Provenance: Under quasi-static equilibrium, end-effector load is
///   the physical cause of joint torque. The FR3 link-side torque sensor
///   at A1 measures the torque that the declared load produces.
///   Direction: cause → observable effect. Observable in D.
///
/// E-02: load → torque_cmd_1 [Coupling]
///   Provenance: End-effector load determines the required torque command
///   at A1 to maintain quasi-static equilibrium.
///
/// E-03: tau_measured_1 → torque_cmd_1 [Continuation]
///   Provenance: The measured joint torque at A1 bounds and informs the
///   commanded torque. Measured state continues into command.
///
/// Interior locus: torque_cmd_1 receives from E-02 and E-03.
/// tau_measured_1 is interior: target(E-01), source(E-03). ABR admits.
pub fn build_relations_exp1() -> DeclaredRelations {
    DeclaredRelations::new(vec![
        DeclaredEdge {
            source: NODE_LOAD.to_string(),
            target: NODE_TAU_MEAS_1.to_string(),
            kind: EdgeKind::Coupling,
            provenance: "Load at end-effector produces measurable torque at A1 \
                         link-side sensor under quasi-static equilibrium".to_string(),
        },
        DeclaredEdge {
            source: NODE_LOAD.to_string(),
            target: NODE_TAU_CMD_1.to_string(),
            kind: EdgeKind::Coupling,
            provenance: "End-effector load determines required torque command at A1".to_string(),
        },
        DeclaredEdge {
            source: NODE_TAU_MEAS_1.to_string(),
            target: NODE_TAU_CMD_1.to_string(),
            kind: EdgeKind::Continuation,
            provenance: "Measured A1 torque bounds and informs the torque command".to_string(),
        },
    ])
}

pub fn build_persistence_exp1() -> PersistenceState {
    PersistenceState::new(RHO_BASE, CHI_0_NM, 3)
}

// ── Experiment 2: A1 + A2 ────────────────────────────────────────────────────

/// Build NodeField for Experiment 2 — A1 + A2.
/// Declaration extended. Same kernel. No new control logic.
pub fn build_nodefield_exp2(
    load_n: f64,
    tau_meas_1_nm: f64,
    tau_meas_2_nm: f64,
) -> NodeField {
    NodeField::new(
        vec![
            NODE_LOAD.to_string(),
            NODE_TAU_MEAS_1.to_string(),
            NODE_TAU_CMD_1.to_string(),
            NODE_TAU_MEAS_2.to_string(),
            NODE_TAU_CMD_2.to_string(),
        ],
        vec![load_n, tau_meas_1_nm, 0.0, tau_meas_2_nm, 0.0],
    )
}

/// Build declared relational topology for Experiment 2.
/// Extends Experiment 1 by declaring A2 edges.
/// Kernel called once over full 5-node, 7-edge topology.
///
/// E-04: load → tau_measured_2 [Coupling]
///   Provenance: Load at end-effector produces measurable torque at A2.
///
/// E-05: tau_measured_1 → tau_measured_2 [Continuation]
///   Provenance: Distal joint torque state (A1) couples proximally to A2.
///   The torque chain is serial: A1 state informs A2 state.
///
/// E-06: load → torque_cmd_2 [Coupling]
///   Provenance: Load determines required torque command at proximal A2.
///
/// E-07: tau_measured_2 → torque_cmd_2 [Continuation]
///   Provenance: Measured A2 torque bounds the A2 torque command.
pub fn build_relations_exp2() -> DeclaredRelations {
    DeclaredRelations::new(vec![
        // A1 edges (identical to Experiment 1)
        DeclaredEdge {
            source: NODE_LOAD.to_string(),
            target: NODE_TAU_MEAS_1.to_string(),
            kind: EdgeKind::Coupling,
            provenance: "Load at end-effector produces measurable torque at A1 \
                         link-side sensor under quasi-static equilibrium".to_string(),
        },
        DeclaredEdge {
            source: NODE_LOAD.to_string(),
            target: NODE_TAU_CMD_1.to_string(),
            kind: EdgeKind::Coupling,
            provenance: "End-effector load determines required torque command at A1".to_string(),
        },
        DeclaredEdge {
            source: NODE_TAU_MEAS_1.to_string(),
            target: NODE_TAU_CMD_1.to_string(),
            kind: EdgeKind::Continuation,
            provenance: "Measured A1 torque bounds and informs the A1 torque command".to_string(),
        },
        // A2 extension edges — declaration grows, codebase does not
        DeclaredEdge {
            source: NODE_LOAD.to_string(),
            target: NODE_TAU_MEAS_2.to_string(),
            kind: EdgeKind::Coupling,
            provenance: "Load at end-effector produces measurable torque at A2 \
                         link-side sensor under quasi-static equilibrium".to_string(),
        },
        DeclaredEdge {
            source: NODE_TAU_MEAS_1.to_string(),
            target: NODE_TAU_MEAS_2.to_string(),
            kind: EdgeKind::Continuation,
            provenance: "Distal A1 torque state couples proximally to A2 in serial chain".to_string(),
        },
        DeclaredEdge {
            source: NODE_LOAD.to_string(),
            target: NODE_TAU_CMD_2.to_string(),
            kind: EdgeKind::Coupling,
            provenance: "End-effector load determines required torque command at A2".to_string(),
        },
        DeclaredEdge {
            source: NODE_TAU_MEAS_2.to_string(),
            target: NODE_TAU_CMD_2.to_string(),
            kind: EdgeKind::Continuation,
            provenance: "Measured A2 torque bounds and informs the A2 torque command".to_string(),
        },
    ])
}

pub fn build_persistence_exp2() -> PersistenceState {
    PersistenceState::new(RHO_BASE, CHI_0_NM, 7)
}

// ── Admissibility check ───────────────────────────────────────────────────────

/// Verify no ring topology exists in a relation set.
/// A ring would mean a node is its own relational predecessor.
pub fn verify_no_ring(relations: &DeclaredRelations) -> bool {
    for edge in &relations.edges {
        if edge.source == edge.target {
            return false;
        }
    }
    // Check for 2-cycles
    for i in 0..relations.edges.len() {
        for j in (i + 1)..relations.edges.len() {
            let a = &relations.edges[i];
            let b = &relations.edges[j];
            if a.source == b.target && a.target == b.source {
                return false;
            }
        }
    }
    true
}

/// Verify interior locus exists for ABR admissibility.
/// A node is interior if it is both a target of some edge and
/// a source of some other edge.
pub fn find_interior_loci(relations: &DeclaredRelations) -> Vec<String> {
    let targets: std::collections::HashSet<&str> =
        relations.edges.iter().map(|e| e.target.as_str()).collect();
    let sources: std::collections::HashSet<&str> =
        relations.edges.iter().map(|e| e.source.as_str()).collect();
    targets.intersection(&sources)
        .map(|s| s.to_string())
        .collect()
}

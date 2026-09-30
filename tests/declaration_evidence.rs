// Declaration Evidence — Test Suite
// Metatron Dynamics, Inc.
// Tags follow kernel reference Section 17 classification.
//
// [IMPL] — implementation executes declared behavior
// [MATH] — declared mathematical property holds
// [OBS]  — observable grounded in FR3 published source
// [CORR] — operator output corresponds to declared target
// [REPR] — determinism across independent runs

use abr_fr3::declaration::*;
use abr_fr3::conventional::*;
use abr_fr3::fr3_geometry::*;
use abr_fr3::operators::*;

// ── FR3 Geometry Observable Tests [OBS] ──────────────────────────────────────

#[test]
fn obs_fr3_max_reach_matches_datasheet() {
    // [OBS] FR3 max reach 855 mm from Datasheet R02212 2.4
    assert!((FR3_MAX_REACH_M - 0.855).abs() < 1e-9,
            "FR3 max reach must be 0.855 m per datasheet");
}

#[test]
fn obs_l1_equals_declared_reach() {
    // [OBS] L1 at neutral config equals max reach — declared from datasheet
    assert!((L1_LEVER_ARM_M - FR3_MAX_REACH_M).abs() < 1e-9,
            "L1 must equal declared reach at neutral configuration");
}

#[test]
fn obs_torque_limit_a1_a4_matches_datasheet() {
    // [OBS] FR3 A1-A4 torque limit 87 N·m from Datasheet R02212 2.4
    assert!((FR3_TORQUE_LIMIT_A1_A4_NM - 87.0).abs() < 1e-9);
}

#[test]
fn obs_declared_config_within_joint_limits() {
    // [OBS] Declared configuration θ₁=0, θ₂=0 within published FR3 limits
    assert!(theta1_in_limits(DECLARED_THETA_1_RAD));
    assert!(theta2_in_limits(DECLARED_THETA_2_RAD));
}

// ── Topology Mathematical Tests [MATH] ───────────────────────────────────────

#[test]
fn math_no_ring_exp1() {
    // [MATH] No ring topology in Experiment 1 declaration
    let rel = build_relations_exp1();
    assert!(verify_no_ring(&rel), "Experiment 1 must have no ring topology");
}

#[test]
fn math_no_ring_exp2() {
    // [MATH] No ring topology in Experiment 2 declaration
    let rel = build_relations_exp2();
    assert!(verify_no_ring(&rel), "Experiment 2 must have no ring topology");
}

#[test]
fn math_interior_locus_exp1() {
    // [MATH] ABR admissibility: interior locus exists in Experiment 1
    let rel = build_relations_exp1();
    let interior = find_interior_loci(&rel);
    assert!(!interior.is_empty(), "Experiment 1 must have an interior locus");
    assert!(interior.contains(&NODE_TAU_MEAS_1.to_string()),
            "tau_measured_1 must be interior locus in Experiment 1");
}

#[test]
fn math_interior_loci_exp2() {
    // [MATH] ABR admissibility: interior loci exist in Experiment 2
    let rel = build_relations_exp2();
    let interior = find_interior_loci(&rel);
    assert!(interior.contains(&NODE_TAU_MEAS_1.to_string()));
    assert!(interior.contains(&NODE_TAU_MEAS_2.to_string()));
}

#[test]
fn math_exp2_extends_exp1_declaration() {
    // [MATH] Experiment 2 has strictly more edges than Experiment 1
    // Declaration grows; codebase does not.
    let rel1 = build_relations_exp1();
    let rel2 = build_relations_exp2();
    assert!(rel2.edges.len() > rel1.edges.len(),
            "Experiment 2 must extend Experiment 1 declaration");
    assert_eq!(rel1.edges.len(), 3);
    assert_eq!(rel2.edges.len(), 7);
}

// ── Implementation Tests [IMPL] ───────────────────────────────────────────────

#[test]
fn impl_m_load_mapping() {
    // [IMPL] M maps payload mass to end-effector force correctly
    let f = m_load_from_mass(FR3_RATED_PAYLOAD_KG);
    let expected = FR3_RATED_PAYLOAD_KG * G_M_S2;
    assert!((f - expected).abs() < 1e-9,
            "M load mapping must equal m × g");
}

#[test]
fn impl_nodefield_exp1_has_correct_labels() {
    // [IMPL] NodeField for Experiment 1 contains declared node labels
    let field = build_nodefield_exp1(10.0, 5.0);
    assert_eq!(field.labels.len(), 3);
    assert!(field.labels.contains(&NODE_LOAD.to_string()));
    assert!(field.labels.contains(&NODE_TAU_MEAS_1.to_string()));
    assert!(field.labels.contains(&NODE_TAU_CMD_1.to_string()));
}

#[test]
fn impl_nodefield_exp2_has_correct_labels() {
    // [IMPL] NodeField for Experiment 2 contains all declared node labels
    let field = build_nodefield_exp2(10.0, 5.0, 3.0);
    assert_eq!(field.labels.len(), 5);
    assert!(field.labels.contains(&NODE_TAU_MEAS_2.to_string()));
    assert!(field.labels.contains(&NODE_TAU_CMD_2.to_string()));
}

#[test]
fn impl_operator_a_produces_nonnegative_values() {
    // [IMPL] Operator A produces non-negative activation values
    let f = build_nodefield_exp1(29.43, 25.17);
    let rel = build_relations_exp1();
    let a_vals = operator_a(&f, &rel, CHI_0_NM);
    assert!(a_vals.iter().all(|&v| v >= 0.0),
            "All A values must be non-negative");
}

#[test]
fn impl_operator_b_bounded_by_rho() {
    // [IMPL] Operator B output is bounded by ρ_base × A when prior state is zero
    let f = build_nodefield_exp1(29.43, 25.17);
    let rel = build_relations_exp1();
    let state = build_persistence_exp1();
    let a_vals = operator_a(&f, &rel, CHI_0_NM);
    let b_vals = operator_b(&a_vals, &state);
    // With zero prior state, B = ρ_base × A
    for (i, (&a, &b)) in a_vals.iter().zip(b_vals.iter()).enumerate() {
        let expected = RHO_BASE * a;
        assert!((b - expected).abs() < 1e-9,
                "B[{}] must equal ρ_base × A[{}] with zero prior state", i, i);
    }
}

#[test]
fn impl_torque_cmd_initialised_to_zero() {
    // [IMPL] Output node torque_cmd initialised to 0.0 before kernel runs
    let field = build_nodefield_exp1(29.43, 25.17);
    assert!((field.get(NODE_TAU_CMD_1) - 0.0).abs() < 1e-9);
}

// ── Correspondence Tests [CORR] ───────────────────────────────────────────────

#[test]
fn corr_conventional_exp1_matches_physics() {
    // [CORR] Conventional treatment matches physics ground truth for Exp1
    // Expected by construction for quasi-static single joint.
    let f = m_load_from_mass(FR3_RATED_PAYLOAD_KG);
    let conv = conventional_exp1(f);
    let gt = physics_ground_truth_exp1(f);
    assert!((conv.tau_a1_nm - gt).abs() < 1e-9,
            "Conventional must match physics ground truth for Exp1");
}

#[test]
fn corr_conventional_exp2_joints_differ() {
    // [CORR] Conventional treatment produces distinct torques at A1 and A2
    // because declared lever arms differ. A1 torque > A2 torque at neutral.
    let f = m_load_from_mass(FR3_RATED_PAYLOAD_KG);
    let conv = conventional_exp2(f);
    let tau_a2 = conv.tau_a2_nm.unwrap();
    assert!(conv.tau_a1_nm > tau_a2,
            "A1 torque must exceed A2 torque at neutral config (L1 > L2)");
}

#[test]
fn corr_relational_exp1_produces_nonzero_output() {
    // [CORR] Relational treatment produces a non-zero torque command at A1
    let f = m_load_from_mass(FR3_RATED_PAYLOAD_KG);
    let tau_meas = f * L1_LEVER_ARM_M;
    let field = build_nodefield_exp1(f, tau_meas);
    let rel = build_relations_exp1();
    let mut state = build_persistence_exp1();
    let (result, _) = run_abr(&field, &rel, &mut state, &[NODE_TAU_CMD_1]);
    let tau_cmd = result.get(NODE_TAU_CMD_1);
    assert!(tau_cmd > 0.0,
            "Relational treatment must produce non-zero torque command");
}

#[test]
fn corr_relational_exp1_within_fr3_limits() {
    // [CORR] Relational torque command at A1 is within FR3 admissibility bounds
    let f = m_load_from_mass(FR3_RATED_PAYLOAD_KG);
    let tau_meas = f * L1_LEVER_ARM_M;
    let field = build_nodefield_exp1(f, tau_meas);
    let rel = build_relations_exp1();
    let mut state = build_persistence_exp1();
    let (result, _) = run_abr(&field, &rel, &mut state, &[NODE_TAU_CMD_1]);
    let tau_cmd = result.get(NODE_TAU_CMD_1);
    assert!(is_admitted_a1_a4(tau_cmd),
            "Relational torque command must be within FR3 A1 ±87 N·m limit");
}

// ── Reproducibility Tests [REPR] ─────────────────────────────────────────────

#[test]
fn repr_exp1_deterministic_across_two_runs() {
    // [REPR] Identical inputs produce identical outputs — two independent runs
    let f = m_load_from_mass(FR3_RATED_PAYLOAD_KG);
    let tau_meas = f * L1_LEVER_ARM_M;

    let field_a = build_nodefield_exp1(f, tau_meas);
    let rel_a = build_relations_exp1();
    let mut state_a = build_persistence_exp1();
    let (result_a, _) = run_abr(&field_a, &rel_a, &mut state_a, &[NODE_TAU_CMD_1]);

    let field_b = build_nodefield_exp1(f, tau_meas);
    let rel_b = build_relations_exp1();
    let mut state_b = build_persistence_exp1();
    let (result_b, _) = run_abr(&field_b, &rel_b, &mut state_b, &[NODE_TAU_CMD_1]);

    assert!((result_a.get(NODE_TAU_CMD_1) - result_b.get(NODE_TAU_CMD_1)).abs() < 1e-12,
            "Experiment 1 must be deterministic across independent runs");
}

#[test]
fn repr_exp2_deterministic_across_two_runs() {
    // [REPR] Experiment 2 is deterministic across two independent runs
    let f = m_load_from_mass(FR3_RATED_PAYLOAD_KG);
    let tm1 = f * L1_LEVER_ARM_M;
    let tm2 = f * L2_LEVER_ARM_M;

    let field_a = build_nodefield_exp2(f, tm1, tm2);
    let rel_a = build_relations_exp2();
    let mut state_a = build_persistence_exp2();
    let (result_a, _) = run_abr(&field_a, &rel_a, &mut state_a,
                                  &[NODE_TAU_CMD_1, NODE_TAU_CMD_2]);

    let field_b = build_nodefield_exp2(f, tm1, tm2);
    let rel_b = build_relations_exp2();
    let mut state_b = build_persistence_exp2();
    let (result_b, _) = run_abr(&field_b, &rel_b, &mut state_b,
                                  &[NODE_TAU_CMD_1, NODE_TAU_CMD_2]);

    assert!((result_a.get(NODE_TAU_CMD_1) - result_b.get(NODE_TAU_CMD_1)).abs() < 1e-12);
    assert!((result_a.get(NODE_TAU_CMD_2) - result_b.get(NODE_TAU_CMD_2)).abs() < 1e-12);
}

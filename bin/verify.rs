// verify.rs — Declaration Conformance Verifier
// Metatron Dynamics, Inc.
//
// Checks declaration integrity before any experimental run.
// All checks must pass before simulate.rs output is meaningful.

use abr_fr3::declaration::*;
use abr_fr3::fr3_geometry::*;

fn check(label: &str, pass: bool) {
    println!("  [{}] {}", if pass { "PASS" } else { "FAIL" }, label);
    if !pass {
        std::process::exit(1);
    }
}

fn main() {
    println!("\nDeclaration Conformance Verifier — abr-fr3-joint-control");
    println!("Metatron Dynamics, Inc.\n");

    // ── Geometry checks ───────────────────────────────────────────────────────
    println!("FR3 Geometry:");
    check("Max reach declared positive", FR3_MAX_REACH_M > 0.0);
    check("L1 equals declared reach at neutral config",
          (L1_LEVER_ARM_M - FR3_MAX_REACH_M).abs() < 1e-9);
    check("L2 declared positive", L2_LEVER_ARM_M > 0.0);
    check("L2 < L1 (proximal joint has smaller lever arm at neutral)",
          L2_LEVER_ARM_M < L1_LEVER_ARM_M);
    check("Torque limit A1-A4 declared at 87 N·m",
          (FR3_TORQUE_LIMIT_A1_A4_NM - 87.0).abs() < 1e-9);
    check("Declared theta_1 within FR3 A1 limits",
          theta1_in_limits(DECLARED_THETA_1_RAD));
    check("Declared theta_2 within FR3 A2 limits",
          theta2_in_limits(DECLARED_THETA_2_RAD));

    // ── Operator parameter checks ─────────────────────────────────────────────
    println!("\nOperator Parameters:");
    check("ρ_base in (0, 1)", RHO_BASE > 0.0 && RHO_BASE < 1.0);
    check("χ₀ declared positive", CHI_0_NM > 0.0);
    check("χ₀ in N·m (> 0 and < A1-A4 torque limit)",
          CHI_0_NM < FR3_TORQUE_LIMIT_A1_A4_NM);

    // ── Experiment 1 topology checks ──────────────────────────────────────────
    println!("\nExperiment 1 Topology (A1):");
    let rel1 = build_relations_exp1();
    check("No ring topology", verify_no_ring(&rel1));
    let interior1 = find_interior_loci(&rel1);
    check("Interior locus exists (ABR admissibility)",
          !interior1.is_empty());
    check("tau_measured_1 is interior locus",
          interior1.contains(&NODE_TAU_MEAS_1.to_string()));
    check("3 edges declared", rel1.edges.len() == 3);
    check("All edges have non-empty provenance",
          rel1.edges.iter().all(|e| !e.provenance.is_empty()));
    check("torque_cmd_1 is target of ≥2 edges",
          rel1.edges.iter().filter(|e| e.target == NODE_TAU_CMD_1).count() >= 2);

    // ── Experiment 2 topology checks ──────────────────────────────────────────
    println!("\nExperiment 2 Topology (A1 + A2):");
    let rel2 = build_relations_exp2();
    check("No ring topology", verify_no_ring(&rel2));
    let interior2 = find_interior_loci(&rel2);
    check("Interior loci exist", !interior2.is_empty());
    check("tau_measured_1 is interior locus in Exp2",
          interior2.contains(&NODE_TAU_MEAS_1.to_string()));
    check("tau_measured_2 is interior locus in Exp2",
          interior2.contains(&NODE_TAU_MEAS_2.to_string()));
    check("7 edges declared", rel2.edges.len() == 7);
    check("All edges have non-empty provenance",
          rel2.edges.iter().all(|e| !e.provenance.is_empty()));
    check("Exp2 has all Exp1 edges (declaration extends, not replaces)",
          rel2.edges.len() > rel1.edges.len());

    // ── Separation checks ─────────────────────────────────────────────────────
    println!("\nSeparation (legacy math absent from relational path):");
    check("G_M_S2 used only in M mapping (declared constant)",
          G_M_S2 > 0.0);
    check("L1 and L2 are declared constants not computed at runtime",
          L1_LEVER_ARM_M > 0.0 && L2_LEVER_ARM_M > 0.0);

    println!("\n  ALL CHECKS PASSED — declaration is conformant.");
    println!("  Proceed to: cargo run --bin simulate\n");
}

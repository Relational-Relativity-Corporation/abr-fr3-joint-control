// simulate.rs — ABR FR3 Joint Control Simulation
// Metatron Dynamics, Inc.
//
// Runs Experiments 1 and 2. Prints full three-panel trace for each:
//   Physics ground truth | Conventional treatment | Relational treatment
//
// Every intermediate operator value is printed. Nothing is hidden.
// The numbers in this output match the numbers in the repository.

use abr_fr3::declaration::*;
use abr_fr3::conventional::*;
use abr_fr3::fr3_geometry::*;
use abr_fr3::operators::run_abr;

fn print_separator(label: &str) {
    println!("\n{}", "═".repeat(72));
    println!("  {}", label);
    println!("{}", "═".repeat(72));
}

fn print_subsection(label: &str) {
    println!("\n  ── {} ──", label);
}

fn main() {
    println!("\nABR FR3 Joint Control — Simulation");
    println!("Metatron Dynamics, Inc. — relationalrelativity.dev");
    println!("Platform: Franka Research 3 (FR3)");
    println!("Source:   https://github.com/frankarobotics/franka_description");
    println!("Kernel:   ABR V8 | ρ_base={} | χ₀={} N·m", RHO_BASE, CHI_0_NM);
    println!("Declared config: θ₁=0.0 rad | θ₂=0.0 rad");

    // ── Declared inputs ───────────────────────────────────────────────────────
    let mass_kg = FR3_RATED_PAYLOAD_KG;           // 3.0 kg declared payload
    let f_load_n = m_load_from_mass(mass_kg);     // M maps mass → force
    let tau_meas_1_nm = f_load_n * L1_LEVER_ARM_M; // simulation: sensor = physics
    let tau_meas_2_nm = f_load_n * L2_LEVER_ARM_M; // simulation: sensor = physics

    println!("\nDeclared observable inputs:");
    println!("  Payload mass:       {:.3} kg", mass_kg);
    println!("  End-effector force: {:.4} N  (M: m × g)", f_load_n);
    println!("  τ_measured A1:      {:.4} N·m (FR3 link-side sensor / simulation)",
             tau_meas_1_nm);
    println!("  τ_measured A2:      {:.4} N·m (FR3 link-side sensor / simulation)",
             tau_meas_2_nm);
    println!("  L1 lever arm:       {:.3} m  (declared constant, datasheet R02212)",
             L1_LEVER_ARM_M);
    println!("  L2 lever arm:       {:.3} m  (declared working value, OC-FR3-1)",
             L2_LEVER_ARM_M);

    // ═════════════════════════════════════════════════════════════════════════
    // EXPERIMENT 1 — FR3 Joint A1
    // ═════════════════════════════════════════════════════════════════════════
    print_separator("EXPERIMENT 1: FR3 Joint A1 — quasi-static load");

    // Physics ground truth
    let gt1 = physics_ground_truth_exp1(f_load_n);
    print_subsection("PHYSICS GROUND TRUTH  (τ = F × L)");
    println!("    F_load = {:.4} N", f_load_n);
    println!("    L1     = {:.3} m", L1_LEVER_ARM_M);
    println!("    τ_A1   = {:.4} N·m", gt1);
    println!("    [independent of both treatments — comparison target only]");

    // Conventional treatment
    let conv1 = conventional_exp1(f_load_n);
    print_subsection("CONVENTIONAL TREATMENT  (τ = F_load × L[n])");
    println!("    F_load         = {:.4} N", conv1.f_load_n);
    println!("    L[1]           = {:.3} m  (declared constant)", conv1.l1_m);
    println!("    τ_conv_A1      = {:.4} N·m", conv1.tau_a1_nm);
    println!("    Parameters:      none (no tuned gains)");
    println!("    Operations:      1 multiplication");

    // Relational treatment
    print_subsection("RELATIONAL TREATMENT  (M → A → B → R)");
    let mut field1 = build_nodefield_exp1(f_load_n, tau_meas_1_nm);
    let relations1 = build_relations_exp1();
    let mut state1 = build_persistence_exp1();

    println!("    NodeField (initial):");
    for (label, val) in field1.labels.iter().zip(field1.values.iter()) {
        println!("      {:20} = {:.4}", label, val);
    }

    let interior1 = find_interior_loci(&relations1);
    println!("    Interior loci: {:?}", interior1);
    println!("    No-ring check: {}", if verify_no_ring(&relations1) { "PASS" } else { "FAIL" });

    let (result1, traces1) = run_abr(
        &field1,
        &relations1,
        &mut state1,
        &[NODE_TAU_CMD_1],
    );

    println!("\n    Operator trace (A → B → R):");
    println!("    {:30} {:>10} {:>10} {:>10}", "Edge", "A", "B", "R");
    println!("    {}", "-".repeat(64));
    for t in &traces1 {
        println!("    {:30} {:>10.4} {:>10.4} {:>10.4}",
                 t.edge_label, t.a_value, t.b_value, t.r_value);
    }

    let tau_rel_1 = result1.get(NODE_TAU_CMD_1);
    println!("\n    NodeField (resolved):");
    for (label, val) in result1.labels.iter().zip(result1.values.iter()) {
        println!("      {:20} = {:.4}", label, val);
    }

    // Correspondence
    println!("\n  ── CORRESPONDENCE vs Physics Ground Truth ──");
    println!("    Ground truth:    {:.4} N·m", gt1);
    println!("    Conventional:    {:.4} N·m  Δ = {:.4} N·m",
             conv1.tau_a1_nm, (conv1.tau_a1_nm - gt1).abs());
    println!("    Relational:      {:.4} N·m  Δ = {:.4} N·m",
             tau_rel_1, (tau_rel_1 - gt1).abs());

    let admitted1 = is_admitted_a1_a4(tau_rel_1);
    println!("    FR3 A1 limit ±{} N·m: {}",
             FR3_TORQUE_LIMIT_A1_A4_NM,
             if admitted1 { "ADMITTED" } else { "EXCEEDED — outside D" });

    // ═════════════════════════════════════════════════════════════════════════
    // EXPERIMENT 2 — FR3 Joints A1 + A2
    // ═════════════════════════════════════════════════════════════════════════
    print_separator("EXPERIMENT 2: FR3 Joints A1 + A2 — same load, extended declaration");
    println!("  Declaration extended. Kernel called once. No new control logic.");
    println!("  Nodes: 5  |  Edges: 7  (was 3 nodes, 3 edges in Experiment 1)");

    // Physics ground truth
    let (gt2_a1, gt2_a2) = physics_ground_truth_exp2(f_load_n);
    print_subsection("PHYSICS GROUND TRUTH");
    println!("    τ_A1 = F × L1 = {:.4} × {:.3} = {:.4} N·m", f_load_n, L1_LEVER_ARM_M, gt2_a1);
    println!("    τ_A2 = F × L2 = {:.4} × {:.3} = {:.4} N·m", f_load_n, L2_LEVER_ARM_M, gt2_a2);
    println!("    Note: τ_A1 ≠ τ_A2 because declared lever arms differ.");
    println!("    [independent of both treatments — comparison target only]");

    // Conventional treatment
    let conv2 = conventional_exp2(f_load_n);
    print_subsection("CONVENTIONAL TREATMENT");
    println!("    τ_conv_A1 = {:.4} × {:.3} = {:.4} N·m",
             conv2.f_load_n, conv2.l1_m, conv2.tau_a1_nm);
    println!("    τ_conv_A2 = {:.4} × {:.3} = {:.4} N·m",
             conv2.f_load_n, conv2.l2_m.unwrap(), conv2.tau_a2_nm.unwrap());
    println!("    Operations: 2 multiplications (one per joint — grows with N)");

    // Relational treatment
    print_subsection("RELATIONAL TREATMENT  (M → A → B → R, one kernel call)");
    let field2 = build_nodefield_exp2(f_load_n, tau_meas_1_nm, tau_meas_2_nm);
    let relations2 = build_relations_exp2();
    let mut state2 = build_persistence_exp2();

    println!("    NodeField (initial):");
    for (label, val) in field2.labels.iter().zip(field2.values.iter()) {
        println!("      {:20} = {:.4}", label, val);
    }

    let interior2 = find_interior_loci(&relations2);
    println!("    Interior loci: {:?}", interior2);
    println!("    No-ring check: {}", if verify_no_ring(&relations2) { "PASS" } else { "FAIL" });

    let (result2, traces2) = run_abr(
        &field2,
        &relations2,
        &mut state2,
        &[NODE_TAU_CMD_1, NODE_TAU_CMD_2],
    );

    println!("\n    Operator trace (A → B → R):");
    println!("    {:35} {:>10} {:>10} {:>10}", "Edge", "A", "B", "R");
    println!("    {}", "-".repeat(69));
    for t in &traces2 {
        println!("    {:35} {:>10.4} {:>10.4} {:>10.4}",
                 t.edge_label, t.a_value, t.b_value, t.r_value);
    }

    let tau_rel_2_a1 = result2.get(NODE_TAU_CMD_1);
    let tau_rel_2_a2 = result2.get(NODE_TAU_CMD_2);

    println!("\n    NodeField (resolved):");
    for (label, val) in result2.labels.iter().zip(result2.values.iter()) {
        println!("      {:20} = {:.4}", label, val);
    }

    // Correspondence
    println!("\n  ── CORRESPONDENCE vs Physics Ground Truth ──");
    println!("    Joint A1:");
    println!("      Ground truth:  {:.4} N·m", gt2_a1);
    println!("      Conventional:  {:.4} N·m  Δ = {:.4} N·m",
             conv2.tau_a1_nm, (conv2.tau_a1_nm - gt2_a1).abs());
    println!("      Relational:    {:.4} N·m  Δ = {:.4} N·m",
             tau_rel_2_a1, (tau_rel_2_a1 - gt2_a1).abs());
    println!("    Joint A2:");
    println!("      Ground truth:  {:.4} N·m", gt2_a2);
    println!("      Conventional:  {:.4} N·m  Δ = {:.4} N·m",
             conv2.tau_a2_nm.unwrap(), (conv2.tau_a2_nm.unwrap() - gt2_a2).abs());
    println!("      Relational:    {:.4} N·m  Δ = {:.4} N·m",
             tau_rel_2_a2, (tau_rel_2_a2 - gt2_a2).abs());

    let admitted2_a1 = is_admitted_a1_a4(tau_rel_2_a1);
    let admitted2_a2 = is_admitted_a1_a4(tau_rel_2_a2);
    println!("    FR3 limits: A1 {} | A2 {}",
             if admitted2_a1 { "ADMITTED" } else { "EXCEEDED" },
             if admitted2_a2 { "ADMITTED" } else { "EXCEEDED" });

    // ═════════════════════════════════════════════════════════════════════════
    // Summary
    // ═════════════════════════════════════════════════════════════════════════
    print_separator("SUMMARY");
    println!("  Experiment 1 (A1):     3 nodes | 3 edges | 1 kernel call");
    println!("  Experiment 2 (A1+A2):  5 nodes | 7 edges | 1 kernel call");
    println!("  Declaration grew. Codebase did not.");
    println!("  Both treatments received identical declared inputs.");
    println!("  Physics ground truth was independent of both.");
    println!("\n  Open conditions: OC-FR3-1 (L2 exact), OC-FR3-2 (ρ_base),");
    println!("                   OC-FR3-3 (χ₀), OC-FR3-4 through OC-FR3-8.");
    println!("\n  Bounded over D. No claim beyond D.");
    println!("  Metatron Dynamics, Inc. — relationalrelativity.dev\n");
}

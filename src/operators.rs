// ABR Kernel V8 — vendored from abr-kernel main
// Unmodified per Repository Protocol.
// Origin: Metatron Dynamics, Inc.

/// NodeField: declared observation vector over D.
/// Each component corresponds to a declared observable node.
#[derive(Debug, Clone)]
pub struct NodeField {
    pub values: Vec<f64>,
    pub labels: Vec<String>,
}

impl NodeField {
    pub fn new(labels: Vec<String>, values: Vec<f64>) -> Self {
        assert_eq!(labels.len(), values.len(), "NodeField: label/value mismatch");
        NodeField { values, labels }
    }

    pub fn get(&self, label: &str) -> f64 {
        let idx = self.labels.iter().position(|l| l == label)
            .unwrap_or_else(|| panic!("NodeField: unknown label '{}'", label));
        self.values[idx]
    }

    pub fn set(&mut self, label: &str, value: f64) {
        let idx = self.labels.iter().position(|l| l == label)
            .unwrap_or_else(|| panic!("NodeField: unknown label '{}'", label));
        self.values[idx] = value;
    }
}

/// DeclaredEdge: a directed relation between two declared nodes.
/// Every edge has declared provenance. No edge without provenance.
#[derive(Debug, Clone)]
pub struct DeclaredEdge {
    pub source: String,
    pub target: String,
    pub kind: EdgeKind,
    pub provenance: String,
}

#[derive(Debug, Clone, PartialEq)]
pub enum EdgeKind {
    Coupling,
    Continuation,
}

/// DeclaredRelations: the complete declared topology for D.
#[derive(Debug, Clone)]
pub struct DeclaredRelations {
    pub edges: Vec<DeclaredEdge>,
}

impl DeclaredRelations {
    pub fn new(edges: Vec<DeclaredEdge>) -> Self {
        DeclaredRelations { edges }
    }

    pub fn edges_incident_to(&self, target: &str) -> Vec<&DeclaredEdge> {
        self.edges.iter().filter(|e| e.target == target).collect()
    }

    pub fn edges_from(&self, source: &str) -> Vec<&DeclaredEdge> {
        self.edges.iter().filter(|e| e.source == source).collect()
    }
}

/// PersistenceState: tracks relational state across operator steps.
#[derive(Debug, Clone)]
pub struct PersistenceState {
    pub rho_base: f64,
    pub chi_0: f64,
    pub edge_activations: Vec<f64>,
}

impl PersistenceState {
    pub fn new(rho_base: f64, chi_0: f64, edge_count: usize) -> Self {
        PersistenceState {
            rho_base,
            chi_0,
            edge_activations: vec![0.0; edge_count],
        }
    }
}

/// OperatorTrace: full intermediate record for every operator step.
/// Nothing is hidden. Every value is preserved.
#[derive(Debug, Clone)]
pub struct OperatorTrace {
    pub edge_label: String,
    pub a_value: f64,
    pub b_value: f64,
    pub r_value: f64,
}

/// Operator A — Attention: computes activation over each declared edge.
/// A(x)[e] = |x[source(e)] - x[target(e)]| / (|x[source(e)]| + chi_0)
pub fn operator_a(
    field: &NodeField,
    relations: &DeclaredRelations,
    chi_0: f64,
) -> Vec<f64> {
    relations.edges.iter().map(|e| {
        let src = field.get(&e.source);
        let tgt = field.get(&e.target);
        let delta = (src - tgt).abs();
        let denom = src.abs() + chi_0;
        delta / denom
    }).collect()
}

/// Operator B — Binding: accumulates relational state at each target node.
/// B(x)[e] = rho_base * A(x)[e] + (1 - rho_base) * prior_activation[e]
pub fn operator_b(
    a_values: &[f64],
    state: &PersistenceState,
) -> Vec<f64> {
    a_values.iter().enumerate().map(|(i, &a)| {
        state.rho_base * a + (1.0 - state.rho_base) * state.edge_activations[i]
    }).collect()
}

/// Operator R — Resolution: produces output at each interior locus.
/// For each target node, R selects the max B value over incident edges
/// and projects it back into the field dimension.
/// R(x)[node] = max B(x)[e] for e incident to node, scaled by source value.
pub fn operator_r(
    field: &NodeField,
    relations: &DeclaredRelations,
    b_values: &[f64],
    target_node: &str,
) -> f64 {
    let incident: Vec<(usize, &DeclaredEdge)> = relations.edges.iter()
        .enumerate()
        .filter(|(_, e)| e.target == target_node)
        .collect();

    if incident.is_empty() {
        return 0.0;
    }

    // Select max B over incident edges
    let (max_idx, _) = incident.iter()
        .max_by(|(i, _), (j, _)| b_values[*i].partial_cmp(&b_values[*j]).unwrap())
        .unwrap();

    let max_b = b_values[*max_idx];
    let source_val = field.get(&relations.edges[*max_idx].source);

    // R projects: source value weighted by relational activation
    // Saturating form: output approaches source as activation increases
    source_val * (1.0 - (-max_b).exp())
}

/// Full ABR kernel pass: A → B → R over declared topology.
/// Returns: updated NodeField at output nodes, full trace.
pub fn run_abr(
    field: &NodeField,
    relations: &DeclaredRelations,
    state: &mut PersistenceState,
    output_nodes: &[&str],
) -> (NodeField, Vec<OperatorTrace>) {
    let a_values = operator_a(field, relations, state.chi_0);
    let b_values = operator_b(&a_values, state);

    // Update persistence state
    state.edge_activations = b_values.clone();

    // Build trace
    let traces: Vec<OperatorTrace> = relations.edges.iter()
        .enumerate()
        .map(|(i, e)| OperatorTrace {
            edge_label: format!("{}->{}", e.source, e.target),
            a_value: a_values[i],
            b_value: b_values[i],
            r_value: 0.0, // filled below per output node
        })
        .collect();

    // Resolve output nodes
    let mut result_field = field.clone();
    let mut full_traces = traces;

    for &out_node in output_nodes {
        let r_val = operator_r(field, relations, &b_values, out_node);
        result_field.set(out_node, r_val);

        // Record R value on incident edges
        for (i, e) in relations.edges.iter().enumerate() {
            if e.target == out_node {
                full_traces[i].r_value = r_val;
            }
        }
    }

    (result_field, full_traces)
}

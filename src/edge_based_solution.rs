use crate::{common::Float, frank_wolfe::SolutionOps};

#[derive(Clone)]
pub struct EdgeBasedSolution {
    edge_flow: Vec<f64>,
    permit_flow: Vec<f64>,
}

impl EdgeBasedSolution {
    pub fn zero(num_edges: usize, num_permits: usize) -> Self {
        EdgeBasedSolution {
            edge_flow: vec![0.0; num_edges],
            permit_flow: vec![0.0; num_permits],
        }
    }

    pub fn num_edges(&self) -> usize {
        self.edge_flow.len()
    }

    pub fn edge_flow(&self) -> &Vec<Float> {
        &self.edge_flow
    }

    pub fn permit_flow(&self) -> &Vec<Float> {
        &self.permit_flow
    }

    pub fn from_vec(edge_flow: Vec<f64>, permit_flow: Vec<f64>) -> Self {
        EdgeBasedSolution {
            edge_flow,
            permit_flow,
        }
    }

    pub fn inner_prod(&self, other: &Self) -> Float {
        self.edge_flow
            .iter()
            .zip(&other.edge_flow)
            .map(|(f1, f2)| f1 * f2)
            .sum::<Float>()
            + self
                .permit_flow
                .iter()
                .zip(&other.permit_flow)
                .map(|(f1, f2)| f1 * f2)
                .sum::<Float>()
    }
}

impl SolutionOps for EdgeBasedSolution {
    fn from_linear_combination(sol1: &Self, scale2: Float, sol2: &Self) -> Self {
        let edge_flow = sol1
            .edge_flow
            .iter()
            .zip(&sol2.edge_flow)
            .map(|(f1, f2)| f1 + scale2 * f2)
            .collect();
        let permit_flow = sol1
            .permit_flow
            .iter()
            .zip(&sol2.permit_flow)
            .map(|(f1, f2)| f1 + scale2 * f2)
            .collect();

        EdgeBasedSolution {
            edge_flow,
            permit_flow,
        }
    }

    fn assign_linear_combination(&mut self, sol1: &Self, scale2: Float, sol2: &Self) {
        for ((f1, f2), f) in sol1
            .edge_flow
            .iter()
            .zip(&sol2.edge_flow)
            .zip(&mut self.edge_flow)
        {
            *f = f1 + scale2 * f2;
        }
        for ((f1, f2), f) in sol1
            .permit_flow
            .iter()
            .zip(&sol2.permit_flow)
            .zip(&mut self.permit_flow)
        {
            *f = f1 + scale2 * f2;
        }
    }

    fn add_scaled(&mut self, scale: Float, other: &Self) {
        for (f_other, f_self) in other.edge_flow.iter().zip(&mut self.edge_flow) {
            *f_self += scale * f_other;
        }
        for (f_other, f_self) in other.permit_flow.iter().zip(&mut self.permit_flow) {
            *f_self += scale * f_other;
        }
    }
}

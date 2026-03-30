use std::mem::swap;

use log::{debug, info, warn};

/// Returns the minimum of two values, if they are comparable, and otherwise returns the second value.
///
/// Specifically, for IEEE-754 floating point numbers, if either value is NaN, the second value is returned.
pub fn partial_min<T: PartialOrd>(a: T, b: T) -> T {
    if a < b { a } else { b }
}

type Float = f64;

/// A solution to the linearized problem at the given gradient to get a search direction
///
/// The linearized problem is of the form
/// ```math
///     min <grad f(x), y>
///     s.t. y in C
/// ```
/// The function returns an optimal solution y, together with the direction y - x,
/// and the inner product of grad f(x) and (y - x).  
pub struct LinearizedSubProblemSolution<Solution> {
    /// An optimal solution to the linearized problem.
    pub solution: Solution,

    /// The direction `y - x` from the current solution `x` to the optimal solution of the linearized problem `y`.
    pub direction: Solution,

    /// The inner product of the gradient at the current solution and the direction, i.e., `<grad f(x), y - x>`.
    ///
    /// The sum of f(x) and this inner product is a new lower bound on the optimal value of the convex program.
    /// Hence, the negated inner product is a lower bound on the optimality gap at point x.
    pub inner_product: Float,
}

pub trait SolutionOps: Clone {
    /// Creates a new solution as the linear combination sol1 + scale2 * sol2
    fn from_linear_combination(sol1: &Self, scale2: Float, sol2: &Self) -> Self;

    /// Assigns self to sol1 + scale2 * sol2.
    fn assign_linear_combination(&mut self, sol1: &Self, scale2: Float, sol2: &Self);

    /// Adds scaled version of other to self: self += scale * other
    fn add_scaled(&mut self, scale: Float, other: &Self);
}

pub trait ConvexProgramInstance<Solution: SolutionOps> {
    /// Given a solution x and a direction v, computes the directional derivative of the function at x along v,
    /// i.e., <grad f(x), v>
    fn directional_derivative(&mut self, at: &Solution, direction: &Solution) -> Float;

    /// Computes the objective value f(x).
    fn compute_objective(&mut self, solution: &Solution) -> Float;

    /// Solves the linearized problem at the given gradient to get a search direction
    ///
    /// The linearized problem is of the form
    /// ```math
    ///     min <grad f(x), y>
    ///     s.t. y in C
    /// ```
    /// The function returns an optimal solution y, together with the direction y - x,
    /// and the inner product of grad f(x) and (y - x). The sum of f(x) and this inner product is a new lower bound
    /// on the optimal value of the convex program. Hence, the negated inner product is a lower bound
    /// on the optimality gap at point x.
    fn solve_subproblem(&mut self, x: &Solution) -> LinearizedSubProblemSolution<Solution>;
}

/// Find a step size alpha that minimizes the objective f(current + alpha * direction) on [0, 1].
///
/// We assume that the objective is convex along this line segment.
pub fn line_search<Solution: SolutionOps, I: ConvexProgramInstance<Solution>>(
    initial: &Solution,
    direction: &Solution,
    instance: &mut I,
) -> Float {
    let derivative_zero_tol: Float = 1e-8;
    let line_search_max_iters: usize = 40;
    let mut low_alpha: Float = 0.0;
    let mut high_alpha: Float = 1.0;

    let mut low_sol = initial.clone();

    // Short circuit using the convexity of the objective along the line segment,
    // since the directional derivative is non-decreasing in alpha.
    // If the directional derivative at "low" is non-negative, "low" is minimal.
    let mut low_deriv = instance.directional_derivative(initial, direction);
    if low_deriv >= 0.0 {
        return 0.0;
    }

    let mut high_sol = initial.clone();
    high_sol.add_scaled(1.0, direction);

    // Analogously, if the directional derivative at "high" is non-positive, "high" is minimal.
    let mut high_deriv = instance.directional_derivative(&high_sol, direction);
    if high_deriv <= 0.0 {
        return 1.0;
    }

    // INVARIANT: directional derivative is negative at low_alpha, and positive at high_alpha.

    let mut mid_sol: Option<Solution> = None;

    for _ in 0..line_search_max_iters {
        let mid_alpha = 0.5 * (low_alpha + high_alpha);
        if let Some(mid_sol1) = &mut mid_sol {
            mid_sol1.assign_linear_combination(initial, mid_alpha, direction);
        } else {
            mid_sol = Some(Solution::from_linear_combination(
                initial, mid_alpha, direction,
            ))
        };
        let mid_sol = mid_sol.as_mut().unwrap();
        let mid_deriv = instance.directional_derivative(mid_sol, direction);

        if -derivative_zero_tol <= mid_deriv && mid_deriv <= 0.0 {
            debug!(
                "Mid derivative {:.6e} is within tolerance, returning mid_alpha = {:.6e}",
                mid_deriv, mid_alpha
            );
            return mid_alpha;
        }

        if mid_deriv > 0.0 {
            high_alpha = mid_alpha;
            if high_deriv < mid_deriv {
                debug!(
                    "Warning: high derivative increased from {:.6e} to {:.6e} when moving high_alpha from {:.6e} to {:.6e}",
                    high_deriv, mid_deriv, high_alpha, mid_alpha
                );
            }
            high_deriv = mid_deriv;
            swap(&mut high_sol, mid_sol);
        } else {
            low_alpha = mid_alpha;
            if low_deriv > mid_deriv {
                debug!(
                    "Warning: low derivative decreased from {:.6e} to {:.6e} when moving low_alpha from {:.6e} to {:.6e}",
                    low_deriv, mid_deriv, low_alpha, mid_alpha
                );
            }
            low_deriv = mid_deriv;
            swap(&mut low_sol, mid_sol);
        }
    }

    low_alpha
}

pub struct FrankWolfeResult<Solution> {
    pub solution: Solution,
    pub num_iterations: usize,
    pub objective_value: Float,
    pub optimality_gap: Float,
    pub relative_optimality_gap: Float,
}

impl<Solution> FrankWolfeResult<Solution> {
    pub fn improve_gap(&mut self, new_gap: Float) {
        if new_gap < self.optimality_gap {
            self.optimality_gap = new_gap;
            self.relative_optimality_gap = new_gap / self.objective_value;
        }
    }

    pub fn set_obj_val(&mut self, new_obj_val: Float) {
        self.optimality_gap = self.optimality_gap - (self.objective_value - new_obj_val);
        self.relative_optimality_gap = self.optimality_gap / new_obj_val;
    }
}

pub fn solve_convex_program<Solution: SolutionOps, I: ConvexProgramInstance<Solution>>(
    initial_solution: Solution,
    instance: &mut I,
    rel_gap_tol: Float,
    max_iterations: usize,
    mut on_step: impl FnMut(&mut I, &Solution),
) -> FrankWolfeResult<Solution> {
    let abs_gap_tol: Float = 1e-8;

    let mut result = FrankWolfeResult {
        num_iterations: 0,
        objective_value: instance.compute_objective(&initial_solution),
        solution: initial_solution,
        optimality_gap: Float::INFINITY,
        relative_optimality_gap: Float::INFINITY,
    };

    loop {
        // Note: result.num_iterations is incremented in [[frank_wolfe_step]].
        if result.num_iterations >= max_iterations {
            info!("Reached maximum number of iterations.");
            break;
        }

        debug!(
            "Before Iteration {}: obj val = {:.6e}, gap = {:.6e}, relative gap = {:.6e}",
            result.num_iterations,
            result.objective_value,
            result.optimality_gap,
            result.relative_optimality_gap
        );

        let linear_solution = instance.solve_subproblem(&result.solution);

        result.improve_gap(-linear_solution.inner_product);

        debug!(
            "During Iteration {}: obj val = {:.6e}, gap = {:.6e}, relative gap = {:.6e}, linear obj val = {:.6e}",
            result.num_iterations,
            result.objective_value,
            result.optimality_gap,
            result.relative_optimality_gap,
            linear_solution.inner_product
        );

        if result.optimality_gap < 0.0 {
            warn!("Warning: negative optimality gap. We *should* be optimal.");
            break;
        }
        if result.optimality_gap < abs_gap_tol {
            info!("Optimal solution found.");
            break;
        }
        if result.relative_optimality_gap.abs() < rel_gap_tol {
            info!("Desired relative optimality gap reached.");
            break;
        }

        let (new_result, step_size) = frank_wolfe_step(
            result,
            instance,
            linear_solution.solution,
            &linear_solution.direction,
        );
        result = new_result;
        on_step(instance, &result.solution);

        if step_size == 0.0 {
            warn!(
                "Warning: step size is zero, but optimality goal not reached. We *should* be optimal."
            );
            break;
        }
    }

    info!(
        "Finished Frank-Wolfe with objective value {:.6e}, gap {:.6e}, relative gap {:.6e}",
        result.objective_value, result.optimality_gap, result.relative_optimality_gap
    );

    result
}

pub fn frank_wolfe_step<Solution: SolutionOps, I: ConvexProgramInstance<Solution>>(
    mut result: FrankWolfeResult<Solution>,
    instance: &mut I,
    mut target_solution: Solution,
    direction: &Solution,
) -> (FrankWolfeResult<Solution>, Float) {
    result.num_iterations += 1;
    let step_size = line_search(&result.solution, direction, instance);
    debug!("step size: {:.6e}", step_size);
    if step_size == 0.0 {
        return (result, step_size);
    }
    if step_size == 1.0 {
        swap(&mut result.solution, &mut target_solution);
    } else {
        result.solution.add_scaled(step_size, direction);
    };
    let new_obj_val = instance.compute_objective(&result.solution);
    if new_obj_val > result.objective_value {
        warn!(
            "Warning: objective increased when moving towards target solution. diff = {:.6e}.",
            new_obj_val - result.objective_value
        );
    }
    result.set_obj_val(new_obj_val);

    debug!(
        "After Iteration {}: obj val = {:.6e}, gap = {:.6e}, relative gap = {:.6e}",
        result.num_iterations,
        result.objective_value,
        result.optimality_gap,
        result.relative_optimality_gap
    );

    (result, step_size)
}

#[cfg(test)]
mod tests {
    use crate::frank_wolfe::{
        ConvexProgramInstance, Float, LinearizedSubProblemSolution, SolutionOps,
        solve_convex_program,
    };

    #[test]
    fn test_solve_convex_program() {
        #[derive(Clone)]
        struct SimpleSolution {
            x: f64,
            y: f64,
        }

        impl SimpleSolution {
            fn inner_prod(&self, other: &Self) -> Float {
                self.x * other.x + self.y * other.y
            }
        }

        impl SolutionOps for SimpleSolution {
            fn from_linear_combination(sol1: &Self, scale2: Float, sol2: &Self) -> Self {
                SimpleSolution {
                    x: sol1.x + scale2 * sol2.x,
                    y: sol1.y + scale2 * sol2.y,
                }
            }

            fn assign_linear_combination(&mut self, sol1: &Self, scale2: Float, sol2: &Self) {
                self.x = sol1.x + scale2 * sol2.x;
                self.y = sol1.y + scale2 * sol2.y;
            }

            fn add_scaled(&mut self, scale: Float, other: &Self) {
                self.x += scale * other.x;
                self.y += scale * other.y;
            }
        }

        struct SimpleInstance;

        impl ConvexProgramInstance<SimpleSolution> for SimpleInstance {
            fn directional_derivative(
                &mut self,
                at: &SimpleSolution,
                direction: &SimpleSolution,
            ) -> Float {
                2.0 * (at.x * direction.x + at.y * direction.y)
            }

            fn compute_objective(&mut self, solution: &SimpleSolution) -> Float {
                solution.x * solution.x + solution.y * solution.y
            }

            fn solve_subproblem(
                &mut self,
                _x: &SimpleSolution,
            ) -> LinearizedSubProblemSolution<SimpleSolution> {
                // The feasible region is the unit ball.
                let direction = SimpleSolution { x: -_x.x, y: -_x.y };
                let norm = (direction.inner_prod(&direction)).sqrt();
                let optimal_point = if norm > 1.0 {
                    SimpleSolution {
                        x: -direction.x / norm,
                        y: -direction.y / norm,
                    }
                } else {
                    SimpleSolution {
                        x: direction.x,
                        y: direction.y,
                    }
                };

                let direction = SimpleSolution::from_linear_combination(&optimal_point, -1.0, _x);

                LinearizedSubProblemSolution {
                    solution: optimal_point.clone(),
                    direction: direction.clone(),
                    inner_product: self.directional_derivative(_x, &direction),
                }
            }
        }

        let initial_solution = SimpleSolution { x: 0.23, y: -0.3 };
        let mut instance = SimpleInstance;

        let final_solution =
            solve_convex_program(initial_solution, &mut instance, 0.0, 100, |_, _| {}).solution;
        assert!(final_solution.x.abs() < 1e-4);
        assert!(final_solution.y.abs() < 1e-4);
    }
}

use std::mem::swap;

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
    fn directional_derivative(&self, at: &Solution, direction: &Solution) -> Float;

    /// Computes the objective value f(x).
    fn compute_objective(&self, solution: &Solution) -> Float;

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
    fn solve_subproblem(&self, x: &Solution) -> LinearizedSubProblemSolution<Solution>;
}

/// Find a step size alpha that minimizes the objective f(current + alpha * direction) on [0, 1].
///
/// We assume that the objective is convex along this line segment.
pub fn line_search<Solution: SolutionOps, I: ConvexProgramInstance<Solution>>(
    initial: &Solution,
    direction: &Solution,
    instance: &I,
) -> Float {
    let derivative_zero_tol: Float = 1e-8;
    let line_search_max_iters: usize = 20;

    let mut low_alpha: Float = 0.0;
    let mut high_alpha: Float = 1.0;

    let mut low_sol = initial.clone();

    // Short circuit using the convexity of the objective along the line segment.:
    // If the directional derivative at "low" is non-negative, "low" is optimal.
    if instance.directional_derivative(initial, direction) > derivative_zero_tol {
        return 0.0;
    }

    let mut high_sol = initial.clone();
    high_sol.add_scaled(1.0, direction);

    // Analogously, if the directional derivative at "high" is non-positive, "high" is optimal.
    let high_deriv = instance.directional_derivative(&high_sol, direction);
    if high_deriv < derivative_zero_tol {
        return 1.0;
    }

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

        if mid_deriv.abs() < derivative_zero_tol {
            return mid_alpha;
        }

        if mid_deriv > 0.0 {
            high_alpha = mid_alpha;
            swap(&mut high_sol, mid_sol);
        } else {
            low_alpha = mid_alpha;
            swap(&mut low_sol, mid_sol);
        }
    }

    0.5 * (low_alpha + high_alpha)
}

pub fn solve_convex_program<Solution: SolutionOps, I: ConvexProgramInstance<Solution>>(
    initial_solution: Solution,
    instance: I,
) -> Solution {
    let max_iterations: usize = 7000;
    let rel_gap_tol: Float = 1e-9;
    let abs_gap_tol: Float = 1e-8;

    let mut cur_solution: Solution = initial_solution;
    let mut cur_obj_val: Float = instance.compute_objective(&cur_solution);
    let mut gap = Float::INFINITY;

    for iteration in 0..max_iterations {
        let relative_gap = gap / cur_obj_val;
        println!(
            "Before Iteration {}: obj val = {:.6e}, gap = {:.6e}, relative gap = {:.6e}",
            iteration, cur_obj_val, gap, relative_gap
        );

        let mut linear_solution: LinearizedSubProblemSolution<Solution> =
            instance.solve_subproblem(&cur_solution);
        gap = partial_min(gap, -linear_solution.inner_product);
        let relative_gap = gap / cur_obj_val;

        println!(
            "During Iteration {}: obj val = {:.6e}, gap = {:.6e}, relative gap = {:.6e}",
            iteration, cur_obj_val, gap, relative_gap
        );

        if gap < 0.0 {
            println!("Warning: negative optimality gap. We *should* be optimal.");
            break;
        }
        if gap < abs_gap_tol {
            println!("Optimal solution found.");
            break;
        }
        if relative_gap.abs() < rel_gap_tol {
            println!("Desired relative optimality gap reached.");
            break;
        }

        let step_size = line_search(&cur_solution, &linear_solution.direction, &instance);
        println!("Line search step size: {:.6e}", step_size);
        if step_size == 0.0 {
            println!(
                "Warning: step size is zero, but optimality goal not reached. We *should* be optimal."
            );
            break;
        }
        if step_size == 1.0 {
            let new_obj_val = instance.compute_objective(&linear_solution.solution);
            let diff = cur_obj_val - new_obj_val;
            if diff < 0.0 {
                println!(
                    "Warning: objective increased when moving to linear solution. diff = {:.6e}.",
                    diff
                );
            }
            gap -= diff;

            swap(&mut cur_solution, &mut linear_solution.solution);
            cur_obj_val = new_obj_val;
        } else {
            cur_solution.add_scaled(step_size, &linear_solution.direction);

            let new_obj_val = instance.compute_objective(&cur_solution);
            let diff = cur_obj_val - new_obj_val;
            if diff < 0.0 {
                println!(
                    "Warning: objective increased when moving to linear solution. diff = {:.6e}.",
                    diff
                );
            }
            gap -= diff;

            cur_obj_val = new_obj_val;
        }
    }

    println!(
        "Finished gradient descent with objective value {:.6e}, gap {:.6e}, relative gap {:.6e}",
        cur_obj_val,
        gap,
        gap / cur_obj_val
    );

    cur_solution
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
                &self,
                at: &SimpleSolution,
                direction: &SimpleSolution,
            ) -> Float {
                2.0 * (at.x * direction.x + at.y * direction.y)
            }

            fn compute_objective(&self, solution: &SimpleSolution) -> Float {
                solution.x * solution.x + solution.y * solution.y
            }

            fn solve_subproblem(
                &self,
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
        let instance = SimpleInstance;

        let final_solution = solve_convex_program(initial_solution, instance);
        assert!(final_solution.x.abs() < 1e-4);
        assert!(final_solution.y.abs() < 1e-4);
    }
}

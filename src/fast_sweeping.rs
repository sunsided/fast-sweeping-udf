use crate::{min3, DistanceField, DistanceFieldAlgorithm, Obstacles};

/// Configuration and implementation of the naive fast sweeping method.
///
/// This algorithm computes distance fields by performing directional sweeps
/// across the grid, propagating distance values from obstacles outward.
#[derive(Debug, Clone, Copy)]
pub struct NaiveFastSweepingMethod {
    step_size: f32,
    max_iterations: usize,
}

impl NaiveFastSweepingMethod {
    /// Creates a new fast sweeping method with the given parameters.
    #[deprecated(since = "0.2.0", note = "use `default()` with builder methods instead")]
    pub fn new(step_size: f32, num_iter: usize) -> Self {
        Self {
            step_size,
            max_iterations: num_iter,
        }
    }

    /// Sets the step size for distance propagation.
    ///
    /// A value of `1.0` means each cell adds 1.0 to the minimum neighbor distance.
    /// Smaller values produce finer distance gradients.
    #[must_use]
    pub const fn with_step_size(mut self, step_size: f32) -> Self {
        self.step_size = step_size;
        self
    }

    /// Sets the maximum number of sweep iterations.
    ///
    /// If set to `0`, the algorithm runs until convergence (no changes detected).
    #[must_use]
    pub const fn with_max_iterations(mut self, max_iterations: usize) -> Self {
        self.max_iterations = max_iterations;
        self
    }

    fn fast_sweeping(&self, distance_field: &mut DistanceField, obstacles: &Obstacles) {
        self.initialize(distance_field, obstacles);
        self.perform_sweeps(distance_field);
    }

    fn initialize(&self, distance_field: &mut DistanceField, obstacles: &Obstacles) {
        for (dist, &is_obstacle) in distance_field.iter_mut().zip(obstacles.iter()) {
            *dist = if is_obstacle {
                0_f32
            } else {
                DistanceField::MAX_DISTANCE
            };
        }
    }

    fn perform_sweeps(&self, distance_field: &mut DistanceField) {
        let max_iter = self.max_iterations;
        let mut iteration = 0;

        loop {
            let mut changed = false;
            changed |= self.sweep_topleft_bottomright(distance_field);
            changed |= self.sweep_bottomright_topleft(distance_field);
            changed |= self.sweep_topright_bottomleft(distance_field);
            changed |= self.sweep_bottomleft_topright(distance_field);

            iteration += 1;
            if !changed || (max_iter > 0 && iteration >= max_iter) {
                break;
            }
        }
    }

    fn sweep_topleft_bottomright(&self, distance_field: &mut DistanceField) -> bool {
        let step_size = self.step_size;
        let height = distance_field.height();
        let width = distance_field.width();
        let mut changed = false;

        for y in 1..height {
            let (current_row, prev_row) = distance_field.get_rows_mut(y - 1);
            let mut carry = current_row[0];

            for x in 1..width {
                let center = current_row[x];
                let up_neighbor = prev_row[x];
                let new_value = min3(center, up_neighbor + step_size, carry + step_size);

                if new_value != center {
                    current_row[x] = new_value;
                    changed = true;
                }
                carry = new_value;
            }
        }
        changed
    }

    fn sweep_bottomright_topleft(&self, distance_field: &mut DistanceField) -> bool {
        let step_size = self.step_size;
        let height = distance_field.height();
        let width = distance_field.width();
        let mut changed = false;

        for y in (0..height - 1).rev() {
            let (current_row, next_row) = distance_field.get_rows_mut(y);
            let mut carry = current_row[width - 1];

            for x in (0..width - 1).rev() {
                let center = current_row[x];
                let down_neighbor = next_row[x];
                let new_value = min3(center, down_neighbor + step_size, carry + step_size);

                if new_value != center {
                    current_row[x] = new_value;
                    changed = true;
                }
                carry = new_value;
            }
        }
        changed
    }

    fn sweep_topright_bottomleft(&self, distance_field: &mut DistanceField) -> bool {
        let step_size = self.step_size;
        let height = distance_field.height();
        let width = distance_field.width();
        let mut changed = false;

        for y in 1..height {
            let (current_row, prev_row) = distance_field.get_rows_mut(y - 1);
            let mut carry = current_row[width - 1];

            for x in (0..width - 1).rev() {
                let center = current_row[x];
                let up_neighbor = prev_row[x];
                let new_value = min3(center, up_neighbor + step_size, carry + step_size);

                if new_value != center {
                    current_row[x] = new_value;
                    changed = true;
                }
                carry = new_value;
            }
        }
        changed
    }

    fn sweep_bottomleft_topright(&self, distance_field: &mut DistanceField) -> bool {
        let step_size = self.step_size;
        let height = distance_field.height();
        let width = distance_field.width();
        let mut changed = false;

        for y in (0..height - 1).rev() {
            let (current_row, next_row) = distance_field.get_rows_mut(y);
            let mut carry = current_row[0];

            for x in 1..width {
                let center = current_row[x];
                let down_neighbor = next_row[x];
                let new_value = min3(center, down_neighbor + step_size, carry + step_size);

                if new_value != center {
                    current_row[x] = new_value;
                    changed = true;
                }
                carry = new_value;
            }
        }
        changed
    }
}

impl Default for NaiveFastSweepingMethod {
    fn default() -> Self {
        Self {
            step_size: 1.0,
            max_iterations: 0,
        }
    }
}

impl DistanceFieldAlgorithm for NaiveFastSweepingMethod {
    fn calculate_distance_field(&self, distance_field: &mut DistanceField, obstacles: &Obstacles) {
        self.fast_sweeping(distance_field, obstacles)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Grid;

    fn create_test_obstacles() -> Obstacles {
        let mut obstacles = Obstacles::new(640, 480);

        for y in 100..200 {
            obstacles.set_at(100, y, true);
        }

        for x in 100..400 {
            obstacles.set_at(x, 200, true);
        }

        for x in 100..200 {
            obstacles.set_at(400 + x, 200 + x, true);
        }

        obstacles
    }

    #[test]
    fn test_basic_sweep() {
        let obstacles = create_test_obstacles();
        let mut distance_field = DistanceField::from(&obstacles);

        let naive = NaiveFastSweepingMethod::default()
            .with_step_size(0.1)
            .with_max_iterations(50);
        naive.calculate_distance_field(&mut distance_field, &obstacles);

        let mut all_finite_or_zero = true;
        for &dist in distance_field.iter() {
            if !dist.is_finite() && dist != f32::INFINITY {
                all_finite_or_zero = false;
            }
        }
        assert!(all_finite_or_zero, "No NaN distances expected");
    }

    #[test]
    fn test_obstacle_cells_have_zero_distance() {
        let mut obstacles = Obstacles::new(10, 10);
        obstacles.set_at(5, 5, true);

        let mut distance_field = DistanceField::from(&obstacles);
        let naive = NaiveFastSweepingMethod::default();
        naive.calculate_distance_field(&mut distance_field, &obstacles);

        assert_eq!(*distance_field.get_at(5, 5), 0.0);
    }

    #[test]
    fn test_empty_grid_converges() {
        let obstacles = Obstacles::new(10, 10);
        let mut distance_field = DistanceField::from(&obstacles);

        let naive = NaiveFastSweepingMethod::default();
        naive.calculate_distance_field(&mut distance_field, &obstacles);

        for &dist in distance_field.iter() {
            assert!(dist.is_infinite(), "Empty grid should retain MAX_DISTANCE");
        }
    }

    #[test]
    fn test_single_obstacle_propagation() {
        let mut obstacles = Obstacles::new(5, 5);
        obstacles.set_at(2, 2, true);

        let mut distance_field = DistanceField::from(&obstacles);
        let naive = NaiveFastSweepingMethod::default();
        naive.calculate_distance_field(&mut distance_field, &obstacles);

        assert_eq!(*distance_field.get_at(2, 2), 0.0);
        assert!(*distance_field.get_at(2, 3) > 0.0);
        assert!(*distance_field.get_at(3, 2) > 0.0);
    }

    #[test]
    fn test_convergence_detection() {
        let obstacles = create_test_obstacles();
        let mut distance_field = DistanceField::from(&obstacles);

        let naive = NaiveFastSweepingMethod::default();
        naive.calculate_distance_field(&mut distance_field, &obstacles);

        let mut distance_field_2 = DistanceField::from(&obstacles);
        let naive_limited = NaiveFastSweepingMethod::default().with_max_iterations(100);
        naive_limited.calculate_distance_field(&mut distance_field_2, &obstacles);

        for (d1, d2) in distance_field.iter().zip(distance_field_2.iter()) {
            assert_eq!(*d1, *d2, "Convergence should match limited run");
        }
    }
}

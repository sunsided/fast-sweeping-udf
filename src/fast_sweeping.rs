use crate::{min3, DistanceField, DistanceFieldAlgorithm, Grid, Obstacles};

pub struct NaiveFastSweepingMethod {
    step_size: f32,
    num_iter: usize,
}

impl DistanceFieldAlgorithm for NaiveFastSweepingMethod {
    fn calculate_distance_field(&self, distance_field: &mut DistanceField, obstacles: &Obstacles) {
        self.fast_sweeping(distance_field, obstacles)
    }
}

impl NaiveFastSweepingMethod {
    pub fn new(step_size: f32, num_iter: usize) -> Self {
        Self {
            step_size,
            num_iter,
        }
    }

    fn fast_sweeping(&self, distance_field: &mut DistanceField, obstacles: &Obstacles) {
        self.initialize(distance_field, obstacles);
        self.perform_sweeps(distance_field);
    }

    fn initialize(&self, distance_field: &mut DistanceField, obstacles: &Obstacles) {
        distance_field
            .iter_mut()
            .zip(obstacles.iter())
            .for_each(|(dist, &is_obstacle)| {
                *dist = if is_obstacle {
                    0_f32
                } else {
                    DistanceField::MAX_DISTANCE
                };
            });
    }

    fn perform_sweeps(&self, distance_field: &mut DistanceField) {
        let num_iter = self.num_iter;
        let step_size = self.step_size;

        let height = distance_field.height();
        let width = distance_field.width();

        for _i in 0..num_iter {
            // First forward sweep (sweeping from top-left to bottom-right)
            for y in 1..height {
                for x in 1..width {
                    let left_neighbor = *distance_field.get_at(x - 1, y);
                    let center = *distance_field.get_at(x, y);
                    let up_neighbor = *distance_field.get_at(x, y - 1);

                    distance_field.set_at(
                        x,
                        y,
                        min3(center, up_neighbor + step_size, left_neighbor + step_size),
                    );
                }
            }

            // Second forward sweep (sweeping from bottom-right to top-left)
            for y in (0..(height - 1)).rev() {
                for x in (0..(width - 1)).rev() {
                    let center = *distance_field.get_at(x, y);
                    let right_neighbor = *distance_field.get_at(x + 1, y);
                    let down_neighbor = *distance_field.get_at(x, y + 1);

                    distance_field.set_at(
                        x,
                        y,
                        min3(
                            center,
                            down_neighbor + step_size,
                            right_neighbor + step_size,
                        ),
                    )
                }
            }

            // Third backward sweep (sweeping from top-right to bottom-left)
            for y in 1..height {
                for x in (0..(width - 1)).rev() {
                    let center = *distance_field.get_at(x, y);
                    let right_neighbor = *distance_field.get_at(x + 1, y);
                    let up_neighbor = *distance_field.get_at(x, y - 1);

                    distance_field.set_at(
                        x,
                        y,
                        min3(center, up_neighbor + step_size, right_neighbor + step_size),
                    )
                }
            }

            // Fourth backward sweep (sweeping from bottom-left to top-right)
            for y in (0..(height - 1)).rev() {
                for x in 1..width {
                    let left_neighbor = *distance_field.get_at(x - 1, y);
                    let center = *distance_field.get_at(x, y);
                    let down_neighbor = *distance_field.get_at(x, y + 1);

                    distance_field.set_at(
                        x,
                        y,
                        min3(center, down_neighbor + step_size, left_neighbor + step_size),
                    )
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
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

        let mut distance_field = DistanceField::from(&obstacles);

        let naive = NaiveFastSweepingMethod::new(0.1, 5);
        naive.calculate_distance_field(&mut distance_field, &obstacles);
    }
}

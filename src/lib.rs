mod distance_field;
mod obstacles;

pub use crate::distance_field::DistanceField;
pub use crate::obstacles::Obstacles;
use std::path::PathBuf;

pub trait Grid {
    type Item;

    fn get_at(&self, x: usize, y: usize) -> &Self::Item;
    fn set_at(&mut self, x: usize, y: usize, value: Self::Item);

    fn iter(&self) -> std::slice::Iter<'_, Self::Item>;
    fn iter_mut(&mut self) -> std::slice::IterMut<'_, Self::Item>;
}

pub trait SavePgm {
    fn save_pgm(&self, file_name: &PathBuf) -> std::io::Result<()>;
}

fn min3(a: f32, b: f32, c: f32) -> f32 {
    a.min(b).min(c)
}

pub fn fast_sweeping(
    distance_field: &mut DistanceField,
    obstacles: &Obstacles,
    step_size: f32,
    num_iter: usize,
) {
    initialize(distance_field, obstacles);
    perform_sweeps(distance_field, step_size, num_iter);
}

fn initialize(distance_field: &mut DistanceField, obstacles: &Obstacles) {
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

fn perform_sweeps(distance_field: &mut DistanceField, step_size: f32, num_iter: usize) {
    for _i in 0..num_iter {
        // First forward sweep (sweeping from top-left to bottom-right)
        for y in 1..distance_field.height() {
            for x in 1..distance_field.width() {
                let left_neighbor = *distance_field.get_at(x - 1, y);
                let center = *distance_field.get_at(x, y);
                let up_neighbor = *distance_field.get_at(x, y - 1);

                distance_field.set_at(
                    x,
                    y,
                    min3(center, up_neighbor + step_size, left_neighbor + step_size),
                )
            }
        }

        // Second forward sweep (sweeping from bottom-right to top-left)
        for y in (0..(distance_field.height() - 1)).rev() {
            for x in (0..(distance_field.width() - 1)).rev() {
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
        for y in 1..distance_field.height() {
            for x in (0..(distance_field.width() - 1)).rev() {
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
        for y in (0..(distance_field.height() - 1)).rev() {
            for x in 1..distance_field.width() {
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

        fast_sweeping(&mut distance_field, &obstacles, 0.1, 5);
    }
}

mod distance_field;
mod fast_sweeping;
mod obstacles;

use std::path::Path;

pub use crate::distance_field::DistanceField;
pub use crate::fast_sweeping::NaiveFastSweepingMethod;
pub use crate::obstacles::Obstacles;

/// Trait for distance field calculation algorithms.
pub trait DistanceFieldAlgorithm {
    fn calculate_distance_field(&self, distance_field: &mut DistanceField, obstacles: &Obstacles);
}

/// Trait abstracting grid access for distance field and obstacle data.
pub trait Grid {
    type Item;

    fn get_at(&self, x: usize, y: usize) -> &Self::Item;
    fn set_at(&mut self, x: usize, y: usize, value: Self::Item);
}

/// Trait for saving grid data as PGM image files.
pub trait SavePgm {
    fn save_pgm<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()>;
}

fn min3(a: f32, b: f32, c: f32) -> f32 {
    a.min(b).min(c)
}

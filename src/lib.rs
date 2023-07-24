mod distance_field;
mod fast_sweeping;
mod obstacles;

use std::path::PathBuf;

pub use crate::distance_field::DistanceField;
pub use crate::fast_sweeping::NaiveFastSweepingMethod;
pub use crate::obstacles::Obstacles;

pub trait DistanceFieldAlgorithm {
    fn calculate_distance_field(&self, distance_field: &mut DistanceField, obstacles: &Obstacles);
}

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

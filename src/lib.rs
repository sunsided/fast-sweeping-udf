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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_min3_basic() {
        assert_eq!(min3(1.0, 2.0, 3.0), 1.0);
    }

    #[test]
    fn test_min3_all_equal() {
        assert_eq!(min3(5.0, 5.0, 5.0), 5.0);
    }

    #[test]
    fn test_min3_negative() {
        assert_eq!(min3(-1.0, 2.0, 3.0), -1.0);
    }

    #[test]
    fn test_min3_infinity() {
        assert_eq!(min3(f32::INFINITY, 1.0, 2.0), 1.0);
    }
}

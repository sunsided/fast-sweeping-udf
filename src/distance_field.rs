use crate::obstacles::Obstacles;
use crate::{Grid, SavePgm};
use std::fs::File;
use std::io::Write;
use std::path::Path;

/// A 2D grid representing computed distance values.
///
/// Each cell contains the distance to the nearest obstacle.
#[derive(Debug, Clone)]
pub struct DistanceField {
    distances: Vec<f32>,
    width: usize,
    height: usize,
}

impl DistanceField {
    /// Maximum distance value used for initialization.
    pub const MAX_DISTANCE: f32 = f32::INFINITY;

    /// Creates a new distance field with the given dimensions.
    ///
    /// All distances are initialized to [`MAX_DISTANCE`](DistanceField::MAX_DISTANCE).
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            distances: vec![Self::MAX_DISTANCE; width * height],
            width,
            height,
        }
    }

    /// Returns the width of the distance field.
    pub const fn width(&self) -> usize {
        self.width
    }

    /// Returns the height of the distance field.
    pub const fn height(&self) -> usize {
        self.height
    }

    /// Returns an iterator over the distance values.
    pub fn iter(&self) -> std::slice::Iter<'_, f32> {
        self.distances.iter()
    }

    /// Returns a mutable iterator over the distance values.
    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, f32> {
        self.distances.iter_mut()
    }

    /// Returns mutable slices for two adjacent rows.
    ///
    /// Returns `(row_y, row_y_plus_1)`. The caller must ensure `y + 1 < height`.
    pub fn get_rows_mut(&mut self, y: usize) -> (&mut [f32], &mut [f32]) {
        let top = y * self.width;
        let bottom = (y + 1) * self.width;
        let (top_part, bottom_part) = self.distances.split_at_mut(bottom);
        (&mut top_part[top..], bottom_part)
    }
}

impl From<&Obstacles> for DistanceField {
    fn from(value: &Obstacles) -> Self {
        Self::new(value.width(), value.height())
    }
}

impl Grid for DistanceField {
    type Item = f32;

    fn get_at(&self, x: usize, y: usize) -> &Self::Item {
        &self.distances[y * self.width + x]
    }

    fn set_at(&mut self, x: usize, y: usize, value: Self::Item) {
        self.distances[y * self.width + x] = value
    }
}

impl SavePgm for DistanceField {
    fn save_pgm<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let mut file = File::create(path)?;

        let max_value: u8 = 255;
        let max_distance = self.iter().fold(0_f32, |acc, &d| acc.max(d));
        let scaler = if max_distance > 0.0 {
            max_value as f32 / max_distance
        } else {
            1.0
        };

        let header = format!("P5\n{} {}\n{}\n", self.width, self.height, max_value);
        file.write_all(header.as_bytes())?;

        for distance in self.iter() {
            let value = (distance * scaler).clamp(0.0, max_value as f32) as u8;
            file.write_all(&[value])?;
        }

        Ok(())
    }
}

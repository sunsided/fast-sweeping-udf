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
        let (row_y_plus_1, _rest) = bottom_part.split_at_mut(self.width);
        (&mut top_part[top..], row_y_plus_1)
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
        let max_distance = self
            .iter()
            .filter(|&d| d.is_finite())
            .fold(0_f32, |acc, &d| acc.max(d));
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn test_new_dimensions() {
        let df = DistanceField::new(10, 20);
        assert_eq!(df.width(), 10);
        assert_eq!(df.height(), 20);
    }

    #[test]
    fn test_new_initial_values() {
        let df = DistanceField::new(5, 5);
        for &d in df.iter() {
            assert_eq!(d, DistanceField::MAX_DISTANCE);
        }
    }

    #[test]
    fn test_width_height() {
        let df = DistanceField::new(7, 13);
        assert_eq!(df.width(), 7);
        assert_eq!(df.height(), 13);
    }

    #[test]
    fn test_iter_length() {
        let df = DistanceField::new(10, 20);
        assert_eq!(df.iter().count(), 200);
    }

    #[test]
    fn test_iter_mut_modifies() {
        let mut df = DistanceField::new(3, 3);
        for d in df.iter_mut() {
            *d = 42.0;
        }
        for &d in df.iter() {
            assert_eq!(d, 42.0);
        }
    }

    #[test]
    fn test_get_rows_mut_adjacent() {
        let mut df = DistanceField::new(4, 5);
        for (i, d) in df.iter_mut().enumerate() {
            *d = i as f32;
        }
        let (row_a, row_b) = df.get_rows_mut(2);
        assert_eq!(row_a.len(), 4, "row_a length");
        assert_eq!(row_b.len(), 4, "row_b length");
        // Verify the rows are adjacent: row_a should be indices 8..12, row_b should be 12..16
        assert_eq!(row_a[0], 8.0);
        assert_eq!(row_b[0], 12.0);
        // Verify modifying one doesn't affect the other
        row_a[0] = 99.0;
        assert_eq!(row_b[0], 12.0);
    }

    #[test]
    fn test_grid_get_set_roundtrip() {
        let mut df = DistanceField::new(5, 5);
        df.set_at(2, 3, 7.5);
        assert_eq!(*df.get_at(2, 3), 7.5);
    }

    #[test]
    fn test_grid_coordinate_mapping() {
        let mut df = DistanceField::new(10, 10);
        df.set_at(1, 2, 99.0);
        assert_eq!(df.distances[2 * 10 + 1], 99.0);

        df.set_at(0, 0, 1.0);
        assert_eq!(df.distances[0], 1.0);

        df.set_at(9, 9, 2.0);
        assert_eq!(df.distances[99], 2.0);
    }

    #[test]
    fn test_from_obstacles() {
        let obs = Obstacles::new(8, 12);
        let df = DistanceField::from(&obs);
        assert_eq!(df.width(), 8);
        assert_eq!(df.height(), 12);
    }

    #[test]
    fn test_save_pgm_creates_file() {
        let df = DistanceField::new(4, 4);
        let path = "test_distance_field_pgm.pgm";
        df.save_pgm(path).unwrap();
        let content = fs::read(path).unwrap();
        let header = std::str::from_utf8(&content[..content.len() - 16]).unwrap();
        assert!(header.starts_with("P5\n4 4\n255\n"));
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn test_save_pgm_scales_values() {
        let mut df = DistanceField::new(3, 3);
        df.set_at(0, 0, 0.0);
        df.set_at(1, 1, 1.0);
        df.set_at(2, 2, 2.0);
        let path = "test_distance_field_scale.pgm";
        df.save_pgm(path).unwrap();
        let content = fs::read(path).unwrap();
        let header = format!("P5\n3 3\n255\n");
        assert!(content.starts_with(header.as_bytes()));
        let data = &content[header.len()..];
        assert_eq!(data.len(), 9);
        assert_eq!(data[0], 0);
        assert_eq!(data[8], 255);
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn test_save_pgm_ignores_infinity() {
        let mut df = DistanceField::new(3, 3);
        df.set_at(0, 0, 0.0);
        df.set_at(1, 1, 1.0);
        let path = "test_distance_field_infinity.pgm";
        df.save_pgm(path).unwrap();
        let content = fs::read(path).unwrap();
        let header_end = content
            .iter()
            .enumerate()
            .filter(|(_, &b)| b == b'\n')
            .map(|(i, _)| i)
            .nth(2)
            .unwrap();
        let data = &content[header_end + 1..];
        assert_eq!(data.len(), 9);
        assert!(data[4] > 0);
        fs::remove_file(path).unwrap();
    }
}

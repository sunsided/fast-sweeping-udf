use crate::{Grid, SavePgm};
use std::fs::File;
use std::io::Write;
use std::path::Path;

/// A 2D grid representing obstacle positions.
///
/// Each cell indicates whether an obstacle is present at that location.
#[derive(Debug, Clone)]
pub struct Obstacles {
    obstacles: Vec<bool>,
    width: usize,
    height: usize,
}

impl Obstacles {
    /// Creates a new obstacle grid with the given dimensions.
    ///
    /// All cells are initially empty (no obstacles).
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            obstacles: vec![false; width * height],
            width,
            height,
        }
    }

    /// Returns the width of the obstacle grid.
    pub const fn width(&self) -> usize {
        self.width
    }

    /// Returns the height of the obstacle grid.
    pub const fn height(&self) -> usize {
        self.height
    }

    /// Returns an iterator over the obstacle values.
    pub fn iter(&self) -> std::slice::Iter<'_, bool> {
        self.obstacles.iter()
    }

    /// Returns a mutable iterator over the obstacle values.
    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, bool> {
        self.obstacles.iter_mut()
    }
}

impl Grid for Obstacles {
    type Item = bool;

    fn get_at(&self, x: usize, y: usize) -> &Self::Item {
        &self.obstacles[y * self.width + x]
    }

    fn set_at(&mut self, x: usize, y: usize, value: Self::Item) {
        self.obstacles[y * self.width + x] = value
    }
}

impl SavePgm for Obstacles {
    fn save_pgm<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let mut file = File::create(path)?;

        let max_value: u8 = 255;

        let header = format!("P5\n{} {}\n{}\n", self.width, self.height, max_value);
        file.write_all(header.as_bytes())?;

        for &is_obstacle in self.iter() {
            let value = (!is_obstacle as u8) * max_value;
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
        let obs = Obstacles::new(10, 20);
        assert_eq!(obs.width(), 10);
        assert_eq!(obs.height(), 20);
    }

    #[test]
    fn test_new_initial_values() {
        let obs = Obstacles::new(5, 5);
        for &b in obs.iter() {
            assert!(!b, "All cells should be empty");
        }
    }

    #[test]
    fn test_width_height() {
        let obs = Obstacles::new(7, 13);
        assert_eq!(obs.width(), 7);
        assert_eq!(obs.height(), 13);
    }

    #[test]
    fn test_iter_length() {
        let obs = Obstacles::new(10, 20);
        assert_eq!(obs.iter().count(), 200);
    }

    #[test]
    fn test_iter_mut_modifies() {
        let mut obs = Obstacles::new(3, 3);
        for b in obs.iter_mut() {
            *b = true;
        }
        for &b in obs.iter() {
            assert!(b);
        }
    }

    #[test]
    fn test_grid_get_set_roundtrip() {
        let mut obs = Obstacles::new(5, 5);
        obs.set_at(2, 3, true);
        assert!(obs.get_at(2, 3));
        obs.set_at(2, 3, false);
        assert!(!obs.get_at(2, 3));
    }

    #[test]
    fn test_grid_coordinate_mapping() {
        let mut obs = Obstacles::new(10, 10);
        obs.set_at(1, 2, true);
        assert!(obs.obstacles[2 * 10 + 1]);

        obs.set_at(0, 0, true);
        assert!(obs.obstacles[0]);

        obs.set_at(9, 9, true);
        assert!(obs.obstacles[99]);
    }

    #[test]
    fn test_save_pgm_creates_file() {
        let obs = Obstacles::new(4, 4);
        let path = "test_obstacles_pgm.pgm";
        obs.save_pgm(path).unwrap();
        let content = fs::read(path).unwrap();
        let header = std::str::from_utf8(&content[..content.len() - 16]).unwrap();
        assert!(header.starts_with("P5\n4 4\n255\n"));
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn test_save_pgm_obstacle_is_dark() {
        let mut obs = Obstacles::new(3, 3);
        obs.set_at(0, 0, true);
        obs.set_at(1, 1, false);
        let path = "test_obstacles_dark.pgm";
        obs.save_pgm(path).unwrap();
        let content = fs::read(path).unwrap();
        let header_end = content.iter().enumerate().filter(|(_, &b)| b == b'\n').map(|(i, _)| i).nth(2).unwrap();
        let data = &content[header_end + 1..];
        assert_eq!(data.len(), 9);
        assert_eq!(data[0], 0);
        assert_eq!(data[4], 255);
        fs::remove_file(path).unwrap();
    }
}

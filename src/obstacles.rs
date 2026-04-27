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

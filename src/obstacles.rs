use crate::{Grid, SavePgm};
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::slice::{Iter, IterMut};

/// Binary map of obstacles.
pub struct Obstacles {
    pub(crate) obstacles: Vec<bool>,
    pub(crate) width: usize,
    pub(crate) height: usize,
}

impl Obstacles {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            obstacles: vec![false; width * height],
            width,
            height,
        }
    }

    pub fn iter(&self) -> Iter<'_, bool> {
        self.obstacles.iter()
    }

    pub fn iter_mut(&mut self) -> IterMut<'_, bool> {
        self.obstacles.iter_mut()
    }
}

impl Grid for Obstacles {
    type Item = bool;

    #[inline(always)]
    fn get_at(&self, x: usize, y: usize) -> &Self::Item {
        &self.obstacles[y * self.width + x]
    }

    #[inline(always)]
    fn set_at(&mut self, x: usize, y: usize, value: Self::Item) {
        self.obstacles[y * self.width + x] = value
    }

    fn iter(&self) -> Iter<'_, Self::Item> {
        self.iter()
    }

    fn iter_mut(&mut self) -> IterMut<'_, Self::Item> {
        self.iter_mut()
    }
}

impl SavePgm for Obstacles {
    fn save_pgm(&self, file_name: &PathBuf) -> std::io::Result<()> {
        let mut file = File::create(file_name)?;

        let max_value: u8 = 255;

        // Write the header
        let header = format!("P5\n{} {}\n{}\n", self.width, self.height, max_value);
        file.write_all(header.as_bytes())?;

        // Convert and write the pixel data as binary
        for &is_obstacle in self.iter() {
            let value = (!is_obstacle as u8) * max_value;
            file.write_all(&[value])?;
        }

        Ok(())
    }
}

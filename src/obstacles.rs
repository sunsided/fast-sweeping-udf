use crate::{Grid, SavePgm};
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::slice::{from_raw_parts_mut, Iter, IterMut};

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

    #[inline(always)]
    pub fn iter(&self) -> Iter<'_, bool> {
        self.obstacles.iter()
    }

    #[inline(always)]
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

    #[inline(always)]
    #[allow(dead_code)]
    fn get_rows_mut(&mut self, y: usize) -> (&mut [Self::Item], &mut [Self::Item]) {
        let top = y * self.width;
        let bottom = (y + 1) * self.width;
        let ptr = self.obstacles.as_mut_ptr();
        unsafe {
            (
                from_raw_parts_mut(ptr.add(top), self.width),
                from_raw_parts_mut(ptr.add(bottom), self.width),
            )
        }
    }

    #[inline(always)]
    fn iter(&self) -> Iter<'_, Self::Item> {
        self.iter()
    }

    #[inline(always)]
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

use crate::obstacles::Obstacles;
use crate::{Grid, SavePgm};
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::slice::{Iter, IterMut};

/// The distance field.
pub struct DistanceField {
    distances: Vec<f32>,
    width: usize,
    height: usize,
}

impl DistanceField {
    pub const MAX_DISTANCE: f32 = f32::INFINITY;

    pub const fn width(&self) -> usize {
        self.width
    }

    pub const fn height(&self) -> usize {
        self.height
    }

    pub fn iter(&self) -> Iter<'_, f32> {
        self.distances.iter()
    }
    pub fn iter_mut(&mut self) -> IterMut<'_, f32> {
        self.distances.iter_mut()
    }
}

impl From<&Obstacles> for DistanceField {
    fn from(value: &Obstacles) -> Self {
        Self {
            distances: vec![0_f32; value.obstacles.len()],
            width: value.width,
            height: value.height,
        }
    }
}

impl Grid for DistanceField {
    type Item = f32;

    #[inline(always)]
    fn get_at(&self, x: usize, y: usize) -> &Self::Item {
        &self.distances[y * self.width + x]
    }

    #[inline(always)]
    fn set_at(&mut self, x: usize, y: usize, value: Self::Item) {
        self.distances[y * self.width + x] = value
    }

    fn iter(&self) -> Iter<'_, Self::Item> {
        self.iter()
    }

    fn iter_mut(&mut self) -> IterMut<'_, Self::Item> {
        self.iter_mut()
    }
}

impl SavePgm for DistanceField {
    fn save_pgm(&self, file_name: &PathBuf) -> std::io::Result<()> {
        let mut file = File::create(file_name)?;

        let max_value: u8 = 255;
        let scaler = (max_value as f32) / self.iter().fold(0_f32, |acc, &d| acc.max(d));

        // Write the header
        let header = format!("P5\n{} {}\n{}\n", self.width, self.height, max_value);
        file.write_all(header.as_bytes())?;

        // Convert and write the pixel data as binary
        for distance in self.iter() {
            let value = (distance * scaler) as u8;
            file.write_all(&[value])?;
        }

        Ok(())
    }
}

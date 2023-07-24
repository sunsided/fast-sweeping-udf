use crate::obstacles::Obstacles;
use crate::{Grid, SavePgm};
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::slice::{from_raw_parts_mut, Iter, IterMut};

/// The distance field.
pub struct DistanceField {
    distances: Vec<f32>,
    width: usize,
    height: usize,
}

impl DistanceField {
    pub const MAX_DISTANCE: f32 = f32::INFINITY;

    #[inline(always)]
    pub const fn width(&self) -> usize {
        self.width
    }

    #[inline(always)]
    pub const fn height(&self) -> usize {
        self.height
    }

    #[inline(always)]
    pub fn iter(&self) -> Iter<'_, f32> {
        self.distances.iter()
    }

    #[inline(always)]
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

    #[inline(always)]
    fn get_rows_mut(&mut self, y: usize) -> (&mut [Self::Item], &mut [Self::Item]) {
        let top = y * self.width;
        let bottom = (y + 1) * self.width;
        let ptr = self.distances.as_mut_ptr();
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

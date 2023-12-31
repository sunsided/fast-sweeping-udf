use fast_sweeping::{
    DistanceField, DistanceFieldAlgorithm, Grid, NaiveFastSweepingMethod, Obstacles, SavePgm,
};
use std::path::PathBuf;

fn main() {
    let mut obstacles = Obstacles::new(640, 480);

    for y in 100..200 {
        obstacles.set_at(100, y, true);
    }

    for x in 100..400 {
        obstacles.set_at(x, 200, true);
    }

    for x in 100..200 {
        obstacles.set_at(400 + x, 200 + x, true);
    }

    let mut distance_field = DistanceField::from(&obstacles);

    println!("Sweeping ...");
    let naive = NaiveFastSweepingMethod::new(0.1, 5);
    naive.calculate_distance_field(&mut distance_field, &obstacles);

    println!("Saving distance field to PGM ...");
    distance_field
        .save_pgm(&PathBuf::from("test-distances.pgm"))
        .unwrap();

    println!("Saving obstacles to PGM ...");
    obstacles
        .save_pgm(&PathBuf::from("test-obstacles.pgm"))
        .unwrap();
}

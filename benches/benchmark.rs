use criterion::{black_box, criterion_group, criterion_main, Criterion};
use fast_sweeping::{
    DistanceField, DistanceFieldAlgorithm, Grid, NaiveFastSweepingMethod, Obstacles,
};

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("640×480, step 0.1, 1 iteration", |b| {
        let mut obstacles = Obstacles::new(640, 480);
        example_obstacles_640x480(&mut obstacles);

        let naive = NaiveFastSweepingMethod::new(0.1, 1);

        b.iter(|| {
            let mut distance_field = DistanceField::from(&obstacles);
            naive.calculate_distance_field(black_box(&mut distance_field), black_box(&obstacles));
        })
    });

    c.bench_function("640×480, step 0.1, 10 iterations", |b| {
        let mut obstacles = Obstacles::new(640, 480);
        example_obstacles_640x480(&mut obstacles);

        let naive = NaiveFastSweepingMethod::new(0.1, 10);

        b.iter(|| {
            let mut distance_field = DistanceField::from(&obstacles);
            naive.calculate_distance_field(black_box(&mut distance_field), black_box(&obstacles));
        })
    });
}

fn example_obstacles_640x480(obstacles: &mut Obstacles) {
    for y in 100..200 {
        obstacles.set_at(100, y, true);
    }

    for x in 100..400 {
        obstacles.set_at(x, 200, true);
    }

    for x in 100..200 {
        obstacles.set_at(400 + x, 200 + x, true);
    }
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

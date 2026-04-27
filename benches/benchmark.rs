use criterion::{black_box, criterion_group, criterion_main, Criterion};
use fast_sweeping::{
    DistanceField, DistanceFieldAlgorithm, Grid, NaiveFastSweepingMethod, Obstacles,
};

fn create_test_obstacles(obstacles: &mut Obstacles) {
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

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("640×480, step 0.1, 1 iteration", |b| {
        let mut obstacles = Obstacles::new(640, 480);
        create_test_obstacles(&mut obstacles);

        let naive = NaiveFastSweepingMethod::default()
            .with_step_size(0.1)
            .with_max_iterations(1);
        let distance_field = DistanceField::from(&obstacles);

        b.iter(|| {
            let mut df = distance_field.clone();
            naive.calculate_distance_field(black_box(&mut df), black_box(&obstacles));
            df
        })
    });

    c.bench_function("640×480, step 0.1, 10 iterations", |b| {
        let mut obstacles = Obstacles::new(640, 480);
        create_test_obstacles(&mut obstacles);

        let naive = NaiveFastSweepingMethod::default()
            .with_step_size(0.1)
            .with_max_iterations(10);
        let distance_field = DistanceField::from(&obstacles);

        b.iter(|| {
            let mut df = distance_field.clone();
            naive.calculate_distance_field(black_box(&mut df), black_box(&obstacles));
            df
        })
    });

    c.bench_function("1280×960, step 0.1, 1 iteration", |b| {
        let mut obstacles = Obstacles::new(1280, 960);
        create_test_obstacles(&mut obstacles);

        let naive = NaiveFastSweepingMethod::default()
            .with_step_size(0.1)
            .with_max_iterations(1);
        let distance_field = DistanceField::from(&obstacles);

        b.iter(|| {
            let mut df = distance_field.clone();
            naive.calculate_distance_field(black_box(&mut df), black_box(&obstacles));
            df
        })
    });

    c.bench_function("1280×960, step 0.1, convergence", |b| {
        let mut obstacles = Obstacles::new(1280, 960);
        create_test_obstacles(&mut obstacles);

        let naive = NaiveFastSweepingMethod::default().with_step_size(0.1);
        let distance_field = DistanceField::from(&obstacles);

        b.iter(|| {
            let mut df = distance_field.clone();
            naive.calculate_distance_field(black_box(&mut df), black_box(&obstacles));
            df
        })
    });

    c.bench_function("640×480, alloc + sweep", |b| {
        let mut obstacles = Obstacles::new(640, 480);
        create_test_obstacles(&mut obstacles);

        let naive = NaiveFastSweepingMethod::default()
            .with_step_size(0.1)
            .with_max_iterations(1);

        b.iter(|| {
            let mut distance_field = DistanceField::from(black_box(&obstacles));
            naive.calculate_distance_field(black_box(&mut distance_field), black_box(&obstacles));
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

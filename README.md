# Fast Sweeping UDF — Distance Fields using the Fast Sweeping Method

A test implementation of the Fast Sweeping Method for calculation of (unsigned) Distance Fields.

To run an example that outputs the distance field into PGM pictures `test-distances.pgm` and `test-obstacles.pgm`, run

```shell
cargo run --example write_pgm
```

This is the outcome. (For better visibility, the distance field shows the square root of the actual distances.)

| Obstacles                                  | Distance Field (sqrt)                                           |
|--------------------------------------------|-----------------------------------------------------------------|
| ![Obstacle Map](readme/test-obstacles.jpg) | ![Resulting Unsigned Distance Field](readme/test-distances.jpg) |

A naive Fast Sweeping implementation can be found in [`src/fast_sweeping.rs`](src/fast_sweeping.rs).

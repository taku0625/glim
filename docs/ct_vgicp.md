# Continuous-time VGICP odometry

`OdometryEstimationCT_VGICP` is a LiDAR-only alternative to `OdometryEstimationCT`.
It keeps the two scan endpoint poses, continuous-time interpolation, deskewing,
velocity prediction and fixed-lag smoothing of CT-GICP.

Build and install `gtsam_points` with `IntegratedCT_VGICPFactor` before building
GLIM. The supplied GLIM revision requires gtsam_points 1.2.0 or later; the supplied
older gtsam_points fork must be updated to a compatible base for a normal joint
build. The new factor itself also supports that older fork.

Select it by setting `global.config_odometry` in your local `config.json` to
`config_odometry_ct_vgicp.json`. This loads `libodometry_estimation_ct_vgicp.so`.
The existing CT-GICP configuration and module remain available.

- `voxel_resolution`: Gaussian voxel edge length in meters (default 1.0).
- `voxel_lru_thresh`: map retention horizon measured in inserted scans.
- Other optimizer and smoother settings follow `config_odometry_ct.json`.

Correspondences use DIRECT1 voxel lookup, matching the CPU `IntegratedVGICPFactor`:
each transformed source point matches the mean and covariance of its containing
voxel. Points in absent voxels do not contribute. This is different from finding
the nearest individual point with iVox; `ivox_min_points_dist` and
`max_correspondence_distance` therefore do not apply. Choose a voxel resolution
large enough to accommodate the expected prediction error.

The map contains deskewed points and covariances transformed into world
coordinates and is updated after each scan optimization. CUDA, a viewer and IMU
measurements are not required.

After installing both libraries, run the synthetic odometry test with:

```bash
cmake -S tests -B build-tests -DCMAKE_PREFIX_PATH="<glim-prefix>;<gtsam_points-prefix>;<gtsam-prefix>"
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

The gtsam_points `BUILD_TESTS=ON` suite also includes `test_ct_vgicp`, which
checks DIRECT1 association, rigid VGICP normal equations, endpoint derivatives and
continuous-motion recovery with deskewing.

The new factor consistently uses GTSAM's `1/2 rᵀWr` nonlinear error and a
`rᵀWr` Hessian constant term. This avoids error-scaling differences between
the older fork and upstream v1.2.0 while reusing their CT pose Jacobians.

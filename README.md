# GTSAM SLAM Vision

## Purpose

This repository contains MATLAB experiments for visual pose estimation and factor-graph SLAM. It explores how camera poses and landmark/tag locations can be estimated from visual measurements, homography-based initialization, and graph optimization.

The repository is intended as a compact public research/engineering artifact: enough structure to understand the learning path without exposing unpublished datasets or private experiment details.

## Relation to State Estimation

This repository supports my research identity by showing factor-graph thinking for robot state estimation. The same estimation ideas can later connect to active scanning, where camera pose, target geometry, and uncertainty affect scan coverage and planning quality.

## Maturity Level

**Current status:** Public MATLAB research/learning artifact.

The repository contains MATLAB scripts and a GTSAM-style toolbox setup for visual pose estimation and factor-graph experiments. It should be treated as a public learning and portfolio artifact, not a complete SLAM system or paper-level implementation.

## Current Runnable Artifact

The repository includes MATLAB scripts such as:

- `main.m`
- `SLAMusingGTSAM.m`
- `estimate_pose.m`
- `plot_With_GTSAM.m`
- `plot_Without_GTSAM.m`
- `visualize_landmarks.m`

Running them may require matching local image/tag observations, calibration assumptions, and a working MATLAB/GTSAM setup.

Open MATLAB from the repository root and run:

```matlab
main
```

If your MATLAB path does not include the toolbox, add it before running the scripts:

```matlab
addpath(genpath('gtsam_toolbox'));
```

## Data Policy

No private datasets are included in this public repository.

No private lab datasets should be added here. If a script depends on local data, document the expected format and keep private data outside the repo.

Future public examples should use synthetic data or data that is clearly allowed to be shared.

## Planned Synthetic Example

- Add a minimal synthetic landmark/tag setup.
- Add a simple camera trajectory with generated observations.
- Run graph optimization on the synthetic observations.
- Save public-safe plots under `results/` or `media/`.
- Clearly state the simplifications and assumptions.

## Limitations

- This is a simplified educational/research prototype, not a complete production SLAM system.
- Dataset paths and calibration assumptions may need adjustment before running on a new machine.
- Robust outlier handling, loop closure, and full sensor-fusion pipelines are not included.
- Noise values and initialization choices are experiment-dependent.
- No private datasets are included in this public repository.

## Features

- MATLAB scripts for homography-based pose estimation.
- Factor-graph formulation using GTSAM-style constraints.
- Projection constraints between world points and image observations.
- Prior factors for anchoring the first camera/tag frame.
- Between-factor style constraints for known tag geometry and frame-to-frame consistency.
- Plotting scripts for comparing estimates with and without graph optimization.

## Roadmap

- [ ] Add a minimal public synthetic example.
- [ ] Add baseline results without graph optimization.
- [ ] Add metrics for reprojection error and trajectory consistency.
- [ ] Add experiment logs for different noise settings.
- [ ] Add a short report explaining the factor graph design.

## Citation / Acknowledgment

This project is based on visual SLAM, homography-based pose estimation, and factor-graph optimization concepts. It uses or references GTSAM-style modeling; please cite GTSAM and any related course, paper, or dataset if you build on this work.

# GTSAM SLAM Vision

## Overview
This repository contains MATLAB experiments for visual pose estimation and factor-graph SLAM. The project explores how camera poses and landmark/tag locations can be estimated from visual measurements, homography-based initialization, and graph optimization.

The repository is intended as a compact public research/engineering artifact: enough structure to understand the method and reproduce the learning path, without exposing unpublished datasets or private experiment details.

## Research/Engineering Motivation
Visual SLAM connects perception, estimation, geometry, and optimization. A robot that moves through an environment must reason about its own pose and the location of observed landmarks under noisy measurements.

This project focuses on the adjustment step: representing assumptions as factors, assigning uncertainty through noise models, and optimizing the resulting graph. The same ideas appear in mobile robotics, visual-inertial navigation, mapping, inspection, and active perception.

## Features
- MATLAB scripts for homography-based pose estimation.
- Factor-graph formulation using GTSAM-style constraints.
- Projection constraints between world points and image observations.
- Prior factors for anchoring the first camera/tag frame.
- Between-factor style constraints for known tag geometry and frame-to-frame consistency.
- Plotting scripts for comparing estimates with and without graph optimization.

## Method
At a high level, the pipeline follows these steps:

1. Estimate an initial camera pose from visual tag observations using homography.
2. Choose a detected tag or frame as the world-coordinate reference.
3. Build an initial factor graph containing camera poses and landmark/tag points.
4. Add priors, projection constraints, known-size constraints, and smoothness assumptions between nearby camera instants.
5. Assign Gaussian noise models to represent confidence in each measurement or constraint.
6. Optimize the graph and visualize the resulting pose/landmark estimates.

The exact experiment configuration is intentionally kept lightweight so the repository can remain a public learning and portfolio artifact.

## Installation
Clone the repository:

```bash
git clone https://github.com/WikiGenius/GTSAM_SLAM_VISION.git
cd GTSAM_SLAM_VISION
```

Requirements:

- MATLAB.
- GTSAM MATLAB toolbox or compatible local setup.
- Image/vision data or tag observations matching the script assumptions.

If your MATLAB path does not include the toolbox, add it before running the scripts:

```matlab
addpath(genpath('gtsam_toolbox'));
```

## Run
Open MATLAB from the repository root and run the main experiment script:

```matlab
main
```

Useful scripts include:

```matlab
SLAMusingGTSAM
estimate_pose
plot_With_GTSAM
plot_Without_GTSAM
visualize_landmarks
```

## Results
The repository includes plotting scripts for visualizing landmark and pose estimates. Add generated plots, screenshots, or comparison figures to `results/` as the experiments are cleaned up.

Suggested result artifacts:

- Estimated camera trajectory.
- Landmark/tag reconstruction plot.
- Before/after optimization comparison.
- Notes on covariance/noise settings.

## Limitations
- This is a simplified educational/research prototype, not a complete production SLAM system.
- Dataset paths and calibration assumptions may need adjustment before running on a new machine.
- Robust outlier handling, loop closure, and full sensor-fusion pipelines are not included.
- Noise values and initialization choices are experiment-dependent.

## Roadmap
- [ ] Add a minimal public dataset or synthetic example.
- [ ] Add baseline results without graph optimization.
- [ ] Add metrics for reprojection error and trajectory consistency.
- [ ] Add experiment logs for different noise settings.
- [ ] Add a short report explaining the factor graph design.

## Citation / Acknowledgment
This project is based on visual SLAM, homography-based pose estimation, and factor-graph optimization concepts. It uses or references GTSAM-style modeling; please cite GTSAM and any related course, paper, or dataset if you build on this work.

# 🚗 Multimodal Classification Network Guided Trajectory Planning for 4WIS Autonomous Parking

[![Paper](https://img.shields.io/badge/IEEE%20IoT%20Journal-Paper-blue)](https://doi.org/10.1109/JIOT.2026.3678248)
[![DOI](https://img.shields.io/badge/DOI-10.1109%2FJIOT.2026.3678248-blue)](https://doi.org/10.1109/JIOT.2026.3678248)
[![Python](https://img.shields.io/badge/Python-3.8.20-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.4.1-EE4C2C?logo=pytorch&logoColor=white)](https://pytorch.org/)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2019a-orange)](https://www.mathworks.com/products/matlab.html)

Official implementation of the paper **“Multimodal Classification Network Guided Trajectory Planning for 4WIS Autonomous Parking Considering Obstacle Attributes”**, published in the *IEEE Internet of Things Journal*.

The proposed framework combines multimodal scene understanding, guided-point-assisted 4WIS hybrid A* search, obstacle-attribute-aware planning, probabilistic risk-field corridors, and optimal-control-based trajectory refinement. It is designed for safe and efficient autonomous parking in narrow, cluttered, and dynamic environments.

<p align="center">
  <strong>🧠 Scene Understanding&nbsp;&nbsp;→&nbsp;&nbsp;📍 Guided Points&nbsp;&nbsp;→&nbsp;&nbsp;🚙 4WIS Planning&nbsp;&nbsp;→&nbsp;&nbsp;🛡️ Risk-Aware Optimization</strong>
</p>

> **Release status:** The source code and detailed running instructions are being organized for public release. Repository-specific commands and paths marked with `TODO` should be updated before publishing the repository.

## 🔍 Overview

Conventional trajectory planners generally avoid every obstacle in the same way, regardless of whether an obstacle is nontraversable, crossable, or safe to drive over at a limited speed. They also underuse the maneuverability offered by four-wheel independent steering (4WIS) vehicles.

This work addresses these limitations through the following pipeline:

1. A **multimodal classification network (MCN)** fuses an RGB scene image, vehicle states, and region-based color features to classify a planning task as easy or hard.
2. For hard tasks, **guided points (GPs)** decompose the original problem into simpler local planning subtasks.
3. A **4WIS hybrid A\*** planner uses Ackermann steering, diagonal movement, and zero-turn rotation as kinematically feasible motion primitives.
4. A **hierarchical obstacle-handling strategy** selects different actions for nontraversable, crossable, and drive-over obstacles.
5. A **probabilistic risk-field-based driving corridor (RFDC)** accounts for the uncertain motion of dynamic obstacles.
6. The initial path is used as a warm start for an **optimal control problem (OCP)** that refines efficiency, comfort, smoothness, and safety.

## ✨ Key Features

- 🧭 **Adaptive GP activation:** the MCN applies GPs only when a scene is sufficiently complex, avoiding unnecessary computation in easy tasks.
- 🧠 **Multimodal scene representation:** ResNet-18 visual features are fused with initial-state, goal-state, and region-distribution features. An auxiliary alignment loss encourages consistency between modalities.
- 🚙 **Full use of 4WIS maneuverability:** Ackermann, diagonal, and zero-turn modes are incorporated into node expansion, with penalties for unnecessary reversals and frequent mode switching.
- 🚧 **Obstacle-attribute-aware decisions:** the planner can avoid, cross, or drive over obstacles according to their attributes.
- 🛡️ **Risk-aware dynamic-obstacle handling:** probabilistic RFDCs convert uncertain collision avoidance into tractable linear safety constraints.
- ⚙️ **Search-then-optimize architecture:** the hybrid A* path provides a kinematically meaningful warm start for nonlinear trajectory optimization.

## 🧩 Method

### 🧠 Multimodal Classification Network

The MCN receives a semantic RGB scene image and vehicle-state information. A ResNet-18 encodes visual features, while multilayer perceptrons encode the initial state, goal state, and color-region proportions. The fused representation predicts whether the planning task is hard or easy.

For a hard task, the GP module is activated. For an easy task, the planner proceeds without GPs to avoid extra computation.

### 📍 Guided-Point Generation

Guided points are constructed from consecutive farthest-visible points along an A* path. When a direction change is required, a gear-shifting point is obtained from clothoid intersections inside a locally expanded free-space rectangle, with the rectangle center used as a fallback.

The GPs guide initial-path generation but are not imposed as additional constraints during OCP optimization.

### 🚙 4WIS Hybrid A* Planning

The search incorporates three vehicle motion modes:

- Ackermann steering;
- diagonal movement;
- zero-turn rotation.

Customized node costs, heuristic terms, and mode-switching penalties encourage short, smooth, and kinematically feasible paths.

### 🚧 Hierarchical Obstacle Handling

Obstacles are divided into three categories:

| Obstacle attribute | Planning action |
| --- | --- |
| Nontraversable | Avoid the obstacle |
| Crossable | Cross when feasible; otherwise avoid |
| Drive-over | Traverse at a constrained speed or avoid |

This hierarchy allows the planner to exploit obstacle attributes instead of conservatively avoiding every object.

### 🛡️ Risk-Aware Trajectory Optimization

Dynamic-obstacle uncertainty is represented through a probabilistic driving risk field. Risk-aware driving corridors provide linear collision constraints for the OCP. The optimization stage uses the initial 4WIS hybrid A* path as a warm start and enforces vehicle kinematics, collision avoidance, boundary conditions, and speed limits over drive-over obstacles.

## 🧪 Experimental Setup

### MCN Training

- Planning-task samples: **2,880**
- Data split: **8:1:1** for training, validation, and testing
- Training epochs: **200**
- Random seeds: **5**
- Optimizer: **Adam**
- Learning rate: **1e-4**
- Batch size: **32**
- Python: **3.8.20**
- PyTorch: **2.4.1**
- torchvision: **0.19.1**
- Training platform: **Ubuntu 20.04.6 LTS**
- GPU: **NVIDIA GeForce RTX 3090, 24 GB**

### Trajectory Planning

- Workspace size: **40 m x 40 m**
- Obstacle-density levels: **low (5)**, **medium (7)**, and **high (9)**
- Obstacle geometry: random convex polygons with **4–7 vertices**
- Obstacle area: uniformly sampled from **5–50 m²**
- Initial/goal heading: uniformly sampled from **[-π, π]**
- MATLAB: **R2019a**
- Nonlinear solver: **IPOPT**, using **MA27** as the linear solver
- Evaluation CPU: **Intel Core i7-7700HQ**

The reported runtime corresponds to an offline research-validation pipeline rather than onboard real-time deployment.

## 📊 Main Results

Across 150 comparative scenarios, the complete method achieved the best reported success rate and the lowest cumulative risk potential among the evaluated methods.

| Method | Success rate | Cumulative risk potential |
| --- | ---: | ---: |
| Hybrid A* | 55.33% | 7.590 |
| FTHA | 70.67% | 7.284 |
| **Ours** | **85.33%** | **4.119** |

The complete method also achieved a path length of **36.121 m**, traversal time of **32.545 s**, maximum jerk of **0.408 m/s³**, average jerk of **0.278 m/s³**, and computation time of **26.803 s**.

### Component-Level Findings

- Adding GPs increased success from **80% to 90%** and reduced mean path length from **29.482 m to 21.570 m** on 50 tasks.
- MCN-based adaptive GP selection reduced computation time from **46.848 s to 38.166 s**, an improvement of approximately **18.5%**.
- Crossable-obstacle handling increased success from **70% to 92%**.
- Drive-over-obstacle handling increased success from **76% to 90%**.
- The probabilistic RFDC reduced cumulative risk potential from **10.141 to 3.235** and increased the minimum obstacle distance from **0.884 m to 1.591 m**.

For complete comparisons, density-wise results, and ablation studies, please refer to the paper.

## 🗂️ Dataset Preparation

Each MCN sample should include:

- a semantic RGB scene image;
- the initial vehicle state;
- the goal vehicle state;
- a binary hard/easy label.

Labels are generated by comparing 4WIS hybrid A* planning with and without GPs. A task is labeled **easy** when planning without GPs succeeds and requires less computation time; otherwise, it is labeled **hard**.

```bash
# TODO: replace with the released data-generation entry point
python <DATA_GENERATION_SCRIPT>.py --config <DATA_CONFIG>
```

> **TODO before release:** provide the dataset directory convention, generated-file format, random-seed options, and a small example dataset or download link.

## 🧠 Training the MCN

```bash
# TODO: replace with the released MCN training entry point
python <MCN_TRAINING_SCRIPT>.py --config <MCN_CONFIG>
```

The default release configuration should reproduce the paper setting of 200 epochs, Adam optimization, a learning rate of `1e-4`, and a batch size of 32.

## 🚘 Running Trajectory Planning

```matlab
% TODO: replace with the released MATLAB demo entry point
run('<PLANNING_DEMO_SCRIPT>.m');
```

A recommended demo should expose the following options:

- scene and obstacle-density selection;
- MCN-guided GP activation;
- Ackermann, diagonal, and zero-turn primitives;
- crossable and drive-over obstacle handling;
- probabilistic RFDC activation;
- OCP trajectory refinement and result visualization.


## 📝 Citation

If this work is useful in your research, please cite:

```bibtex
@article{teng2026multimodal,
  author  = {Jingjia Teng and Yang Li and Yougang Bian and Manjiang Hu and Yingbai Hu and Guofa Li and Jianqiang Wang},
  title   = {Multimodal Classification Network Guided Trajectory Planning for 4WIS Autonomous Parking Considering Obstacle Attributes},
  journal = {IEEE Internet of Things Journal},
  volume  = {13},
  number  = {12},
  pages   = {26666--26681},
  year    = {2026},
  doi     = {10.1109/JIOT.2026.3678248}
}
```

## 📄 License

Add a software license before the public release and describe any separate terms that apply to data, pretrained weights, or third-party solvers. The paper PDF remains subject to IEEE copyright and is not covered by the source-code license.

## 📬 Contact

For questions about the paper or implementation, please contact:

- **Yang Li** (corresponding author): [lyxc56@gmail.com](mailto:lyxc56@gmail.com)
- **Jingjia Teng**: [tengjingjia@foxmail.com](mailto:tengjingjia@foxmail.com)

## 🙏 Acknowledgments

This work was supported by the National Natural Science Foundation of China under Grant **52302493**.

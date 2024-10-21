
# SceneFlowFusion

### Overview

This repository contains the MATLAB code and resources associated with the paper: **"Camera Motion Estimation from RGB-D-Inertial Scene Flow"** presented at the **CVPRW 2024**.

In this project, we propose a novel approach to estimate camera motion by leveraging RGB-D and inertial data fusion. Our method extracts **scene flow** from RGB-D images and combines it with **inertial measurements** to accurately estimate the 6-DoF motion of a camera in real-time.

### Paper

You can read the full paper [here](https://openaccess.thecvf.com/content/CVPR2024W/VISOD/papers/Cerezo_Camera_Motion_Estimation_from_RGB-D-Inertial_Scene_Flow_CVPRW_2024_paper.pdf).

### Key Contributions

1. **RGB-D and Inertial Fusion**: We present a novel method that combines visual and inertial data to estimate camera motion.
2. **Scene Flow Estimation**: We leverage dense scene flow computed from RGB-D data to obtain 3D motion information.
3. **Improved Accuracy**: Our fusion approach improves accuracy in comparison to visual-only and inertial-only methods.
4. **Real-time Performance**: The method is designed to operate in real-time on standard hardware setups.

### Dataset

We used the [ICL-NUIM](http://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html) and [OpenLoris](https://lifelong-robotic-vision.github.io/dataset/overview) datasets to validate our approach. These datasets contain synchronized RGB-D and inertial data suitable for SLAM and motion estimation tasks.

### Installation

To run the code in this repository, follow the steps below:

```bash
# Clone the repository
git clone https://github.com/username/repo-name.git
cd repo-name
```

### Usage

After setting up MATLAB and dependencies, you can run the demo with:

```matlab
run demo.m
```

### Code Structure

- `src/`: Core implementation of the motion estimation algorithm.
- `data/`: Placeholder for RGB-D and inertial data.
- `scripts/`: Utility scripts for dataset handling and visualization.
- `README.md`: Project description and usage instructions.

### Citation

If you use this code in your research, please cite:

```
@inproceedings{cerezo2024camera,
  title={Camera Motion Estimation from RGB-D-Inertial Scene Flow},
  author={Samuel Cerezo and Javier Civera},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition Workshops (CVPRW)},
  year={2024}
}
```

### License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

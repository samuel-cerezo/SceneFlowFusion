
# SceneFlowFusion

### Overview

This repository contains the MATLAB code and resources associated with the paper: **"Camera Motion Estimation from RGB-D-Inertial Scene Flow"** presented at the **CVPRW 2024**.

In this project, we propose a novel approach to estimate camera motion by leveraging RGB-D and inertial data fusion. we introduce a novel formulation for camera motion estimation that integrates RGB-D images and inertial data through scene flow. Our goal is to accurately estimate the camera motion in a rigid 3D environment, along with the state of the inertial measurement unit (IMU). Our
proposed method offers the flexibility to operate as a multiframe optimization or to marginalize older data, thus effectively utilizing past measurements. To assess the performance of our method, we conducted evaluations using both synthetic data from the ICL-NUIM dataset and real data sequences from the OpenLORIS-Scene dataset. Our results show that the fusion of these two sensors enhances the accuracy of camera motion estimation when compared to using only visual data.

### Paper

You can read the full paper [here](https://openaccess.thecvf.com/content/CVPR2024W/VISOD/papers/Cerezo_Camera_Motion_Estimation_from_RGB-D-Inertial_Scene_Flow_CVPRW_2024_paper.pdf).

### Key Contributions

1. **RGB-D and Inertial Fusion**: We present a novel method that combines visual and inertial data to estimate camera motion.
2. **Scene Flow Estimation**: We leverage dense scene flow computed from RGB-D data to obtain 3D motion information.
3. **Improved Accuracy**: Our fusion approach improves accuracy in comparison to visual-only and inertial-only methods.

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
run main.mlx
```
### Results

The qualitative results for motion estimation are shown in an office scene consisting of a computer on a desk and luminaires. A point cloud is extracted from the RGB-D data, and the scene flow is displayed with red arrows representing velocity at each point. A zoomed-in view highlights that the approach estimates a smooth flow even in textureless areas like the background wall.

![sceneFlow](/images/sf.png)

### Code Structure

- `src/`: Core implementation of the motion estimation algorithm.
- `scripts/`: Utility scripts for dataset handling and visualization.
- `README.md`: Project description and usage instructions.

### Citation

If you use this code in your research, please cite:

```
@INPROCEEDINGS{10677884,
  author={Cerezo, Samuel and Civera, Javier},
  booktitle={2024 IEEE/CVF Conference on Computer Vision and Pattern Recognition Workshops (CVPRW)}, 
  title={Camera Motion Estimation from RGB-D-Inertial Scene Flow}, 
  year={2024},
  volume={},
  number={},
  pages={841-849},
  keywords={Visualization;Three-dimensional displays;Measurement units;Motion estimation;Sensor fusion;Cameras;Sensors;Motion estimation;RGB-D-inertial navigation},
  doi={10.1109/CVPRW63382.2024.00089}}
```

### License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

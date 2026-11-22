---
dataset_info:
  license: other
  license_name: chingmu-terms
  license_link: LICENSE
  language: ["en", "zh"]
  pretty_name: "ChingMu Robot Motion Dataset"
  tags:
    - motion-capture
    - humanoid-robotics
    - imitation-learning
    - optical-mocap
    - bvh
    - dexterous-hands
    - whole-body-control
  size_categories: 1M<n
  configs:
    - config_name: metadata
      default: true
      data_files:
        - split: train
          path: "metadata/index.csv"
    - config_name: samples
      data_files:
        - split: train
          path: "samples/**/*"
---

## 🆕 Open-Source Release: Unitree G1 Retargeted Data   

**!!We are releasing 1000 hours of robot-ready motion trajectories retargeted to the Unitree G1 humanoid. All data is provided in CSV format under the samples/ directory. Please indicate the source of the data when using it: from Chingmu.**   







# ChingMu 1000-Hour Embodied Motion Dataset

> High-precision **optical motion capture** data for humanoid robots, dexterous hands, embodied AI, and virtual production.  

| | |
|---|---:|
| **Duration** | **1000+ hours** @ 120 Hz |
| **Scenarios** | 15+ real-world scenes |
| **Tasks** | 500+ standardized tasks |
| **Objects** | 200+ tracked props (6D pose) |
| **Modalities** | Skeleton · Object 6D · Video · Labels |
| **Formats** | BVH · Retargeted CSV · NPZ |

✅ **Access note:** This dataset is fully open and publicly accessible.  

---

## Key Features 

- **Optical ground truth** – sub-mm accuracy, 120 fps, no estimation errors.    

- **Robot-ready** – pre-retargeted to Unitree G1; custom retargeting available.  

- **Real-world diversity** – 15+ scenarios, 500+ tasks, 200+ objects.  

- **Multi-modal** – full-body skeleton, finger motion, object pose, multi-view video, semantic labels.  

- **Quality assured** – every take passes automated cleaning + manual inspection; quality flags provided.  


---

## Dataset Summary

ChingMu 1000H is an optical motion capture dataset designed for training and validating embodied AI and humanoid robot controllers. It covers full-body skeleton, finger articulation, object 6D pose, multi-view video, and semantic labels across 15+ real-world scenarios (industrial, household, retail, healthcare, logistics, agriculture, performance). All data is cleaned, quality-assessed, and robot-retargeted.

---

## Data Format Specifications 

| Component | Format | Details |
|---|---|---|
| Raw motion | `.bvh` | Y-up, 120 fps, YXZ rotation, mm, 22 joints |
| Retargeted trajectories | `.csv` | Root position (mm), quaternion, joint angles (rad) |
| Object 6D pose | `.csv` | Position (mm) + quaternion, 120 Hz |
| Multi-view video | `.mp4` | 2-3 cameras, co-registered |
| Semantic labels | `.csv` | Task, scenario, action, object |


## 🎥 Preview Video 

Watch a short demonstration of the motion capture data in action:  

<video src="https://github.com/ChingmuData/MotionDecode/raw/refs/heads/main/assets/video/SC01_render_V02_LQ_1.mp4" controls autoplay muted loop>
  Your browser does not support the video tag.
</video>

*Demonstration of full-body motion capture with real-time skeleton overlay and object tracking.*  

### Intended Uses
- Imitation learning / motion policy training for humanoids

- Dexterous manipulation datasets (hand-object interaction)

- Motion generation & retrieval (text/motion cross-modal)

- Sim-to-real validation (MuJoCo via retargeted trajectories)

- Virtual production & animation reference

---

### Full Taxonomy (abridged)

- **Locomotion** → walk, jog, crouch-walk...

- **Manipulation (whole-body)** → shelf-pick-place...

- **Dexterous Hand** → pinch, precision-grasp...

- **Tool Use** → screwdriver, wrench...

- **Object Interaction** → door-open/close...

- **Social / Contact** → handoff-object...、

- **Performance** → dance, martial-arts...


👀 **Try it live:** Use the **Dataset Preview** panel at the top of this page to filter and explore the actual index table. Select the `metadata` config to browse available takes.  

> ℹ️ The full index with all rows is best viewed locally. Download [`metadata/index.csv`](https://huggingface.co/datasets/ZIHLING/Chingmu-RobotData/resolve/main/metadata/index.csv) to open in Excel or pandas for complete filtering.  

---

## 🖥️ Interactive Showcase

Visit our dedicated showcase website for interactive demos, comparison videos, and detailed visualizations:  

[![Visit Showcase](https://img.shields.io/badge/🌐-Visit_Showcase_Website-2ea44f?style=for-the-badge)](https://chingmudata.github.io/MotionDecode/)

*Includes: trailer video, modality breakdowns, robot retargeting comparisons, and more.*  

---


## Quick Start

```bash
pip install huggingface_hub
```

```python
from huggingface_hub import hf_hub_download

repo_id = "CMRobot/MotionDecode"
file_path = hf_hub_download(
    repo_id=repo_id,
    filename="samples/1.1.Basic_Movement_Category/1.1.1.High_Dynamic_Movement/1.1.1.1.Standing_High_Jump/BM_Standing_High_Jump_00001.csv",
    repo_type="dataset",
    local_dir="./samples"
)
print(f"Downloaded: {file_path}")
```

---

## Quality & Limitations

**Quality controls:** marker swap correction, gap-filling (≤6 frames), foot skating detection, manual review. Flags: `pass`, `warning`, `fail`.  

**Accuracy:** joint error <1mm, object pose ±2mm / ±0.5°, temporal sync <1 frame.  

**Limitations:** performer age skew (20–40), object accuracy varies with marker cluster size.  

---

## Get Full Dataset 

The entire dataset is publicly available here. If you have any questions about the dataset or would like to know more information, please contact us through the following channels:  

- **For Chinese users:** Scan the QR code below to contact us via WeChat, and include in the remarks the name of your organization, your name, and the main purpose.   

<img src="https://github.com/ChingmuData/MotionDecode/raw/refs/heads/main/assets/code_20260817.jpg" width="30%" alt="alt text">

**For international users:** Join our Discord community     

[![Discord](https://img.shields.io/badge/Discord-Join_Community-5865F2?style=for-the-badge&logo=discord&logoColor=white)](https://discord.gg/gAzgFqYDr9)

Alternatively, you can click the **"Request access"** button on the right side of this page to automatically gain download permissions for the complete dataset.  

Or email us at: **MotionDecode@chingmu.com**  

We look forward to collaborating with researchers and industry partners!  

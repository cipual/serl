# SERL ON AUBO

**repo:[rail-berkeley/serl: SERL: A Software Suite for Sample-Efficient Robotic Reinforcement Learning](https://github.com/rail-berkeley/serl)**

## Major updates
#### 2025/4/10
由于在训练中会不可避免地碰到插座造成设定位置偏差，在插座旁边贴上Aruco码，进行插座位姿的实时矫正。

#### 2025/3/19
可以在训练时候进行人工干预：按下‘p'键暂停后输入干预的步数进行人工干预。

有别于Spacemouse，人工干预由软件产生：用程序控制到达目标点的过程中进行采样，动作增量作为专家数据取代策略网络输出。

## Setup
1. **Setup AuboServer**
   
    ```bash
    conda activate serl
    cd ~/serl
    python serl_robot_infra/robot_servers/aubo_server.py
    ```
    
2. **Setup AuboRosController:**
   
    ```bash
    roslauch my_aubocontroller impetance.launch load_gripper:=true
    ```

## Overview

1.**insertion**

![insert](./Serl-Test/docs/images/insert.gif)

## Citation

If you use this code for your research, please cite our paper:

```bibtex
@misc{luo2024serl,
      title={SERL: A Software Suite for Sample-Efficient Robotic Reinforcement Learning},
      author={Jianlan Luo and Zheyuan Hu and Charles Xu and You Liang Tan and Jacob Berg and Archit Sharma and Stefan Schaal and Chelsea Finn and Abhishek Gupta and Sergey Levine},
      year={2024},
      eprint={2401.16013},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

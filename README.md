# SERL ON AUBO

**repo: [rail-berkeley/serl: SERL: A Software Suite for Sample-Efficient Robotic Reinforcement Learning](https://github.com/rail-berkeley/serl)**

## Major updates

#### 2025/3/19
可以在训练时候进行人工干预：按下‘p'键暂停后输入干预的步数进行人工干预。

有别于Spacemouse，人工干预由软件产生：用程序控制到达目标点的过程中进行采样，动作增量作为专家数据取代策略网络输出。

## Setup
1. **启动AuboServer**

    ```bash
    conda activate serl
    cd ~/serl
    python serl_robot_infra/robot_servers/aubo_server.py
    ```

2. **启动AuboRosController:**

    ```bash
    roslaunch my_aubo_controller impedance.launch load_gripper:=true
    ```

3. **手动夹取插头：**

    关闭夹爪：

    ```bash
    curl -X POST http://127.0.0.1:5000/close_gripper
    ```
    打开夹爪：

    ```bash
    curl -X POST http://127.0.0.1:5000/open_gripper
    ```

4. **确定目标位置：**
    拖动机械臂至目标地点后获取此时的末端位姿：

      ```bash
      curl -X POST http://127.0.0.1:5000/getpos_euler
      ```
    修改以下几个文件的TARGET_POSE：
    `serl_robot_infra/franka_env/envs/peg_env/config.py line14`
    `rospy-aubo-gripper-controller/src/my_aubo_controller/scripts/create_demo.py line82`
    ​`rospy-aubo-gripper-controller/src/my_aubo_controller/scripts/goto_target.py line34`

    重启AuboRosController：
    ```bash
    roslaunch my_aubo_controller impedance.launch load_gripper:=false
    ```

5. **制作演示数据：**
   
      ```bash
      python examples/async_peg_insert_drq/record_aubo_demo.py
      ```
      
5. **开始训练：**
    需要修改`--demo_path`和`--checkpoint_path`
      ```bash
      bash examples/async_peg_insert_drq/run_actor.sh
      bash examples/async_peg_insert_drq/run_learner.sh
      ```
5. **评估模型：**
    `--checkpoint_path`和`--eval_checkpoint_step`修改成需要评估的模型
      ```bash
      bash examples/async_peg_insert_drq/run_actor_eva.sh
      ```


## Overview

1.**insertion**

![insert](./docs/images/insert.gif)

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

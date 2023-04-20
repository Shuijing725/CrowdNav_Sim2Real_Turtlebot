# CrowdNav_Sim2Real
This repository contains the sim2real procedure and code for our paper titled "Intention Aware Robot Crowd Navigation with Attention-Based Interaction Graph" in ICRA 2023. 
In sim2real, we adapted a people detector and SLAM from previous works, and transfered a simulated crowd navigation policy to a TurtleBot2i without any real-world training.   
For more details, please refer to the [project website](https://sites.google.com/view/intention-aware-crowdnav/home) and [arXiv preprint](https://arxiv.org/abs/2203.01821).
For experiment demonstrations, please refer to the [youtube video](https://www.youtube.com/watch?v=nxpxhF019VA).

<img src="/figures/3humans.gif" width="350" />  
<img src="/figures/4humans.gif" width="350" />  

## System overview
### Hardware
- Host computer:
  - CPU: Intel i7-9700 @ 3GHz
  - GPU: Nvidia RTX 2080
  - Memory: 32GB
- Turtlebot2i:
  - On-board computer: Nvidia Jetson Xavier
  - Lidar: RP-Lidar A3
  - Tracking Camera: Intel Realsense T-265
  - Mobile base: Kobuki base
### Software
The host computer and the turtlebot communicates through ROS by connecting to the same WiFi network.
- Host computer:
  - OS: Ubuntu 20.04
  - Python version: 3.8.10
  - Cuda version: 11.5
  - ROS version: Noetic (**our code WILL NOT WORK with lower versions of ROS on host computer**)
- Turtlebot2i:
  - OS: Ubuntu 18.04
  - Python version: 3.8
  - Cuda version: cuda is not needed
  - ROS version: Melodic


## Setup

### Turtlebot
1. Create a catkin workspace
```
mkdir ~/catkin_ws
cd catkin_ws
mkdir -p src
catkin_make
cd src
```

2. Install ROS packages into your workspace
```
cd ~/catkin_ws/src
# turtlebot2
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_interactions.git

# kobuki
git clone https://github.com/yujinrobot/kobuki_msgs.git

# RP-Lidar
git clone https://github.com/Slamtec/rplidar_ros.git

cd ~/catkin_ws
catkin_make
```

### Host computer
1. Create a catkin workspace
```
mkdir ~/catkin_ws
cd catkin_ws
mkdir -p src
catkin_make
cd src
```

2. Install ROS packages into your workspace
```
cd ~/catkin_ws/src
# turtlebot2
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_interactions.git

# kobuki
git clone https://github.com/yujinrobot/kobuki_msgs.git

# to use lidar for SLAM
git clone https://github.com/surfertas/turtlebot2_lidar.git
git clone https://github.com/SteveMacenski/slam_toolbox/tree/noetic-devel.git

# people detector
git clone https://github.com/VisualComputingInstitute/2D_lidar_person_detection.git

cd ~/catkin_ws
catkin_make
```

3. In `catkin_ws/src/2D_lidar_person_detection/dr_spaam_ros/config/topics.yaml` line 14, change `/segway/scan_multi` to `/person_pts` to remove static obstacles from the input scans of people detector

4. Place `findloc_bgrm.py` into `catkin_ws/src/2D_lidar_person_detection`

5. Download the virtual environment from [this link](https://drive.google.com/file/d/1a9FMezC1henRTCmBy-S6hyN_lAGHDZKw/view?usp=sharing). Create an identical virtual environment in your computer.

6. Connect the robot and host computer to the same WiFi network. In `tb2.bash`, change `ROS_MASTER` to the IP address of the robot, change `ROS_IP` to the IP address of the host computer.
   - Skip this step and all steps related to `source tb2.bash` if you're running everything on a robot's on-board computer.

## Run the code
### Training
- Clone the [crowd navigation repo](https://github.com/Shuijing725/CrowdNav_Prediction_AttnGraph)
- Modify the configurations.
  1. Modify the configurations in `crowd_nav/configs/config.py` and `arguments.py` following instructions [here](https://github.com/Shuijing725/CrowdNav_Prediction_AttnGraph#training)

  2. In `crowd_nav/configs/config.py`, set 
     - `action_space.kinematics = "unicycle"`
     - adjust `sim.circle_radius`, `sim.arena_size`, and `sim.human_num` based on your real environment
     
- After you change the configurations, run
  ```
  python train.py 
  ```
- The checkpoints and configuration files will be saved to the folder specified by `output_dir` in `arguments.py`.

### Testing
1. First, test the trained policy in simulation following instructions [here](https://github.com/Shuijing725/CrowdNav_Prediction_AttnGraph#testing), make sure the results are satisfactory (success rate is at least around 90%)
2. Create a map of the real environment using SLAM:  
   a. [Turtlebot] Launch the mobile base:
      ```
      source catkin_ws/devel/setup.bash
      roslaunch turtlebot2i_bringup minimal.launch
      ```
      If 'no data stream, is kobuki turned on?' shows up even if the base is fully charged, we recommend unplugging the USB wire of LiDAR and restarting the Turtlebot's onboard computer.

   b. [Turtlebot] Plug in the USB wire of LiDAR, launch the LiDAR:
      ```
      source catkin_ws/devel/setup.bash && sudo chmod 666 /dev/ttyUSB0 && sudo chmod 666 /dev/ttyUSB1 && sudo chmod 666 /dev/ttyUSB2 
      roslaunch rplidar.launch
      ```
   c. [Host computer] Launch SLAM and navigation
      ```
      source ~/tb2.bash
      source ~/catkin_ws/devel/setup.bash
      roslaunch turtlebot_navigation laser_gmapping_demo.launch 
      ```
   d. [Host computer] Launch rviz
      ```
      source ~/tb2.bash
      source ~/catkin_ws/devel/setup.bash
      roslaunch turbot_rviz nav.launch
      ```
   e. [Host computer] Launch robot teleoperation
      ```
      source ~/tb2.bash 
      roslaunch turtlebot_teleop keyboard_teleop.launch
      ```
   f. [Host computer] Teleoperate the robot around the environment until you are satisfied with the map, save the map by 
      ```
      rosrun map_server map_saver -f ~/map
      ```
      In your home directory, you will see two files: `map.yaml` and `map.pgm`.

3. Then, test the trained policy in real turtlebot in the mapped environment:
   - [Turtlebot] Launch the mobile base (see Step 2a)

   - [Turtlebot] Launch the LiDAR (see Step 2b)

   - [Host computer] Launch localization and navigation
     ```
     source ~/tb2.bash
     source ~/catkin_ws/devel/setup.bash
     roslaunch turtlebot_navigation laser_amcl_demo.launch map_file:=$HOME/map.yaml
     ```
     This step is ready if the terminal shows "odom received".

   - [Host computer] Launch rviz (see Step 2d)  
     To calibrate localization, use "2D pose estimate" to correct the initial pose of robot, and then use "2D navigation" to navigate the robot around until the localization particles converge. 
   - [Host computer] To filter out the static obstacles on the map and improve the people detection,
        ```
     source ~/tb2.bash
     source ~/catkin_ws/devel/setup.bash
     cd ~/catkin_ws/src/2D_lidar_person_detection 
     python findloc_bgrm.py path_to_the_map_created_in_step2
     ```  
   - [Host computer] Run the DR-SPAAM people detector:
     ```
     source ~/tb2.bash
     source ~/catkin_ws/devel/setup.bash
     source ~/virtual_envs/tb2/bin/activate # activate the virtual environment created in Setup -> Host computer -> Step 6
     roslaunch dr_spaam_ros dr_spaam_ros.launch
     ```  
   - [Turtlebot] Launch the Realsense T265 camera using `t265.launch` from this repo
     ```
     roslaunch t265.launch
     ```
   - [Host computer] Cd into the crowd navigation repo, 
     - in `trained_models/your_output_dir/arguments.py`, change `env-name` to `'rosTurtlebot2iEnv-v0'`
     - in `trained_models/your_output_dir/configs/config.py`, change configurations under `sim2real` if needed
     - then run 
       ```
       python test.py 
       ```
     Type in the goal position following the terminal output, and the robot will execute the policy if everything works.

## To-Do list
1. The robot localization from LiDAR and T265 are redundant. Remove the dependency on T265 later.

## Disclaimer
1. We only tested our code in the above listed hardware and software settings. It may work with other robots/versions of software, but we do not have any guarantee.  

2. If the RL training does not converge, we recommend starting with an easier setting (fewer humans, larger circle radius, larger robot speed, etc) and then gradually increase the task difficulty during training.

3. The performance of our code can vary depending on the choice of hyperparameters and random seeds (see [this reddit post](https://www.reddit.com/r/MachineLearning/comments/rkewa3/d_what_are_your_machine_learning_superstitions/)). 
Unfortunately, we do not have time or resources for a thorough hyperparameter search. Thus, if your results are slightly worse than what is claimed in the paper, it is normal. 
To achieve the best performance, we recommend some manual hyperparameter tuning.


## Citation
If you find the code or the paper useful for your research, please cite the following papers:
```
@inproceedings{liu2022intention,
  title={Intention Aware Robot Crowd Navigation with Attention-Based Interaction Graph},
  author={Liu, Shuijing and Chang, Peixin and Huang, Zhe and Chakraborty, Neeloy and Hong, Kaiwen and Liang, Weihang and Livingston McPherson, D. and Geng, Junyi and Driggs-Campbell, Katherine},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2023}
}

@inproceedings{liu2020decentralized,
  title={Decentralized Structural-RNN for Robot Crowd Navigation with Deep Reinforcement Learning},
  author={Liu, Shuijing and Chang, Peixin and Liang, Weihang and Chakraborty, Neeloy and Driggs-Campbell, Katherine},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2021},
  pages={3517-3524}
}
```

## Credits
Other contributors:  
[Peixin Chang](https://github.com/PeixinC)  
[Kaiwen Hong](https://www.linkedin.com/in/kaiwen-hong-524520141/?locale=en_US)   
[Eric Liang](https://www.linkedin.com/in/weihang-liang-5147a014a/)  

Part of the code is based on the following repositories:  

[1] S. Liu, P. Chang, W. Liang, N. Chakraborty, and K. Driggs-Campbell, "Decentralized Structural-RNN for Robot Crowd Navigation with Deep Reinforcement Learning," in IEEE International Conference on Robotics and Automation (ICRA), 2019, pp. 3517-3524. (Github: https://github.com/Shuijing725/CrowdNav_DSRNN)  

[2] Z. Huang, R. Li, K. Shin, and K. Driggs-Campbell. "Learning Sparse Interaction Graphs of Partially Detected Pedestrians for Trajectory Prediction," in IEEE Robotics and Automation Letters, vol. 7, no. 2, pp. 1198–1205, 2022. (Github: https://github.com/tedhuang96/gst)  

## Contact
If you have any questions or find any bugs, please feel free to open an issue or pull request.

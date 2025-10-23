
# Project description

This project is developed under Ros2 Humble version.

Now it only includes a crab model description in `description` folder. 

```
ros2 launch crab_model gazebo.launch.py
```

Use launch command could visualize the crab model in `Gazebo` and `Rviz`

There is a file named `ikpy.ipynb` that requires `urdf` formate as input to calculate the Forward Kinematic and inverse kinematics of one leg/arm.

It is only applicable to one leg, so for the scenario where the robot's legs are not the same size needs to consider more common tool.

## 1. Aim

The aim of this project is to control a crab model to move following a 3d trajectory formed by Japanese spider crab.

## 2. RoadMap

- [x] Design underwater experiment to collect 2d videos for 3 views
    - **Background:** Camera Calibration
    - **Project files:** 

- [x] Deeplabcut 2D prediction
    - **Background:** Machine Learning Configuration
    - **Project files:** In Red Dragon

- [ ] 3D reconstruction
    - **Background:** Triangulation Method
    - **Project files:** Go to Acinoset_Adapated (also in Red Dragon)

- [ ] Locomotion analysis and control

## 3. Documentation

1. Camera Calibration
    - A calibration guideline in Google Drive
    - OpenCV official calibration documentation 

2. Machine Learning
    - Deeplabcut official documentation
    - Read `DLC_Installation_User_Guide.ipynb` files in `documents/tutorials` folder 

3. 3D reconstrcuction
    - Read `Acinoset_Installation.ipynb` files in `documents/tutorials` folder 
    - Projective transformation lecture notes from [Stanford lectures  CS231A](https://web.stanford.edu/class/cs231a/): The details about camera calibration and camera matrix
    - Videos tutorials from Columbia Uni Prof. Shree Nayar [Firsr Principle of CV](https://fpcv.cs.columbia.edu/): Algorithm of estimating 3d structure from 2 arbitary views.

## 4. Discussion

### 4.1 Concept

1. Transfer Learning: Retraining a pre-trained network with small samples


### 4.2 Todo

- [ ] Fix the index error in acinoset triangulation

- [ ] Analyze the camera calibration results



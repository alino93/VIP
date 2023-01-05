# VIP
Vector-based Inertial Poser for human posture estimation from sparse IMUs

[![Watch the video](https://github.com/alino93/VIP/blob/main/assets/Annotation%202023-01-04%20215912.jpg)](https://github.com/alino93/VIP/blob/main/assets/SMPLPlot.mp4)

ORB-SLAM3 is originally from [here](https://github.com/UZ-SLAMLab/ORB_SLAM3). This modified version is forked from [here](https://github.com/jahaniam/orbslam3_docker). 
Articulate utils and physics version is inspired from [here](https://vcai.mpi-inf.mpg.de/projects/PhysCap/data/physcap.pdf) and [here](https://github.com/Xinyu-Yi/PIP).

You can get access to the SMPL codes and models from [here](https://smpl.is.tue.mpg.de/).

You can get download the MoVi Motion Capture dataset from [here](https://www.biomotionlab.ca/movi/).

## Requirements
We tested our codes with `python 3.7` and 'Matlab 2021a' on ubuntu 18.04.6. 

You should install these packages for python `tensorflow scipy pytorch chumpy vctoolkit open3d pybullet qpsolvers cvxopt`.

You need these toolboxes for Matlab `Control System, Signal Processing, Aerospace, Robotics System, Navigation`.

You also need to compile and install [rbdl](https://github.com/rbdl/rbdl) with python bindings and the urdf reader addon.

## Compilation and Running ORB-SLAM3 Docker
Install docker:
* [Install docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
* If curl install gives you an error, try:
 - `sudo snap install curl`
* To run using docker use `sudo docker` or create a [user group](https://docs.docker.com/engine/install/linux-postinstall/)

Steps to compile the Orbslam3 on the sample dataset:

- `cd ~ # an arbitrary folder outside your ROS workspace`
- `git clone https://github.com/alino93/orbslam3_docker.git`
- `cd orbslam3_docker` 
- `./download_dataset_sample.sh`
- `./build_container_cpu.sh` or `build_container_cuda.sh` depending on your machine.

** If build fails, you need more cpu cores and it closes processes duiring the build. To avoid that you can modify `build.sh` and build from inside the docker:
- `sudo docker exec -it orbslam3 bash`
- `sudo nano build.sh`
- remove all `-j` after `make -j` to limit using one core during the build.
- build: `chmod +x build.sh
          ./build.sh`
          
## Compilation and Running VIP

First clone the repo:
`git clone https://github.com/alino93/VIP.git`

Then run the example code by executing:
`cd VIP`
`./main.sh`

### Notes

Feel free to play with options in `dynamics.py` to test different costs and constraints.

Run `mainWSLAM.sh` instead of `main.sh` to see the results with ORBSLAM3. You should put your YAML camera calibration file and the dataset in the docker.  

Set `debug=True/False` in `runPhysics.py` to show the pybullet visualization of the physics optimizer.

Set `Line 244` in `dynamics.py` to `True` to visualize the ground reaction forces.

Adjust the physics hyperparameters in `physics_parameters.json`



## Citation

If you find the project helpful, please consider citing us:

```
@InProceedings{,
  author = {Nouriani Ali, Rajamani Rajesh, McGovern Robert},
  title = {Vector-based Inertial Poser (VIP): A vector-based and physics-aware human motion tracking system using sparse IMU sensors},
  booktitle = {},
  month = {May},
  year = {2023}
}
```

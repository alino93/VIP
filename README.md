# VIP
Vector-based Inertial Poser for human posture estimation from sparse IMUs

ORB-SLAM3 is originally from [here](https://github.com/UZ-SLAMLab/ORB_SLAM3). This modified version is forked from [here](https://github.com/jahaniam/orbslam3_docker). 
Articulate utils and physics version is inspired from [here](https://vcai.mpi-inf.mpg.de/projects/PhysCap/data/physcap.pdf) and [here](https://github.com/Xinyu-Yi/PIP).

## Requirements


## Compilation and Running
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

#!/bin/bash

#cd ~/orbslam3_docker

sudo docker exec orbslam3 /bin/bash -c "cd /ORB_SLAM3"
sudo docker exec orbslam3 /bin/bash -c "./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/RC.yaml Examples/Monocular/dataset2/"

#!/bin/bash

echo "Estimate IMU orientation ... Run Observer"
python3 runExtData.py

echo "Estimate limb angles ... Run LSTM Net"
python3 runNet.py

echo "Estimate position of joints and ZUPT from Geometry ... Run Matlab code"
python3 runMatlab.py

echo "[Optional] Run camera module ... Run ORB-SLAM3"
sudo docker exec orbslam3 /bin/bash -c "cd /ORB_SLAM3"
sudo docker exec orbslam3 /bin/bash -c "./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/RC.yaml Examples/Monocular/dataset2/"

echo "Optimize the results and apply physical constraints ... Run Physics Optimization"
python3 runPhysics.py

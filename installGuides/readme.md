##Install Order for Jetson TX1
1. Flash Jetpack on Jetson TX1
2. Preinstall script on Jetson TX1
3. installRealSenseR200 install to Jetson TX1
4. installJetPackSSD to install the filesystem on to the SSD
5. Preinstall script on SSD Jetpack
6. installRealSenseR200 install to SSD Jetpack (for safe measure)
7. installKernelModulesToSSD
8. installROS install to SSD Jetpack
9. Install CAR ROS package and dependencies
10. Follow steps in installRealSenseR200ROS install to SSD Jetpack
11. Follow steps in installXV_11  install to SSD Jetpack
12. Create swapspace from createSwap
13. Follow steps in installTensorFlowCUDA for SSD Jetpack
14. Follow steps in installTensorFlow for SSD Jetpack
15. (Optional) compile OpenCV 3.1 with CUDA and Gtreamer support from source
<br/>

##Install order Host PC
1. installROS_HOSTPC.sh
2. installCUDA_CUDNN
3. installTensorFlow


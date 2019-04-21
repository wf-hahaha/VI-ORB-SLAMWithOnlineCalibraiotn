echo "launching mono-VI-ORB-SLAM"

./Examples/Monocular/mono_euroc_VI Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC_VI.yaml ~/dataset/V1_03_difficult/mav0/imu0/data.csv ~/dataset/V1_03_difficult/mav0/cam0/data.csv ~/dataset/V1_03_difficult/mav0/cam0/data V1_03_difficult

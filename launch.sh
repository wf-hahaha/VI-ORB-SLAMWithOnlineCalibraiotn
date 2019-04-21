echo "launching stereo-VI-ORB-SLAM"

./Examples/Stereo/stereo_euroc_VI Vocabulary/ORBvoc.bin Examples/Stereo/EuRoC_VI.yaml ~/dataset/V1_01_easy/mav0/imu0/data.csv ~/dataset/V1_01_easy/mav0/cam0/data.csv ~/dataset/V1_01_easy/mav0/cam0/data ~/dataset/V1_01_easy/mav0/cam1/data V1_01_easy

cmake_minimum_required(VERSION 2.8)
project(relevos)

find_package(qibuild)

# Create a executable named relevos
# with the source file: main.cpp
qi_create_bin(relevos "main.cpp" "NaoVision.h" "NaoVision.cpp" "NaoMovement.h" "NaoMovement.cpp")

qi_use_lib(relevos OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC)
qi_use_lib(relevos ALCOMMON ALPROXIES)
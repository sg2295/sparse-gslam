# Efficient 2D Graph SLAM for Sparse Sensing

This repository contains code for `Efficient 2D Graph SLAM for Sparse Sensing`. The paper can be accessed [here](paper/iros2022.pdf).

THIS SPARSE GSLAM REPOSITORY HAS BEEN EXTENDED AND MODIFIED BY SERGIOS GAVRIILIDIS AS FOLLOWS:
- A new data provider `EV3DataProvider` that provides calibration and digital signal processing methods for scans gathered by an ultrasonic sensor.
- A new `thin_runner.launch` file to enable SparseGSLAM to run on embedded platforms (this runner offers no visualization functionalities, instead capturing relevant ROS topics using ROSBAG).
- A new `map_viz.launch` file that enables the visualization of the SLAM results produced by `thin_runner.launch`.
- Disabled real-time rendering of pose and landmark graphs, including the publication of submap markers, false loop-closure candidate reporting and other non-essential information produced by `log_runner.launch`.
- Various minor modifications in the form of compiler optimizations (e.g. use of `std::array` over `std::vector` where allowed, use of static-casting and `constexpr` wherever applicable) to improve efficiency.
- Modified this README.md file to include new instructions for use with the EV3Rover data acqusition module.

## Installation
- Recommended: Ubuntu 20.04 with ROS noetic OR Ubuntu 18.04 with ROS melodic
- C++14-compatible compiler (preferably GNU)

## Prerequisites
1. Install ROS dependencies:
```
sudo apt install ros-melodic-jsk-rviz-plugins ros-melodic-navigation ros-melodic-joy
```
2. Build & install [Google Cartographer](https://google-cartographer.readthedocs.io/en/latest/)
3. Clone the libg2o-release repository (https://github.com/ros-gbp/libg2o-release):
    - checkout release/{your-ros-distro}/libg2o branch
    - in config.h.in, `#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 0` (edges and vertices are managed by SparseGSLAM)
    - in CMakeLists.txt, set `BUILD_WITH_MARCH_NATIVE` to `ON`
    - update links with `sudo ldconfig`
4. Clone this repository and build
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```
## Running
On the embedded platform (SLAM logic):
```bash
roslaunch sparse_gslam thin_runner.launch dataset:=ev3
scp src/sparse_gslam/bagfiles/out.bag <destination/modern/computer/with/gui>
```
On a computer with GUI capabilities (map and trajectory visualization - MapViz):
```bash
roslaunch sparse_gslam map_viz.launch
```

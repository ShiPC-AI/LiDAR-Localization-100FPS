# LiDAR-Localization-100FPS

 LiDAR-Localization-100FPS  is a map-aided and template descriptor-based LiDAR global localization method.
 
# How to use?
Before run, you must generate a (1) **map point cloud** and (2) **map candidate point cloud**.
## 1. Requirement
```
CMake
PCL
OpenCV
```
## 2. Build the project
```
mkdir build && cd build
cmake .. 
make -j4
```
## 3. Make map descriptors (only executed once before localization)
```
./map_process/make_map_squares
```
## 4. Run global localization
```
./run/run_global_localization
```
# Publication

Shi P, Li J, Zhang Y. LiDAR localization at 100 FPS: A map-aided and template descriptor-based global method[J]. International Journal of Applied Earth Observation and Geoinformation, 2023, 120: 103336.

BibTex file:
@article{shi2023lidar,
  title={LiDAR localization at 100 FPS: A map-aided and template descriptor-based global method},
  author={Shi, Pengcheng and Li, Jiayuan and Zhang, Yongjun},
  journal={International Journal of Applied Earth Observation and Geoinformation},
  volume={120},
  pages={103336},
  year={2023},
  publisher={Elsevier}}

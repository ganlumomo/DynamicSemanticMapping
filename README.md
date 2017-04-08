# DynamicSemanticMapping
Scene Flow Propagation for Semantic Mapping and Object Discovery in Dynamic Street Scenes

### Required Dependencies:
* OctoMap https://octomap.github.io/
* Eigen

### Install:
* git clone https://github.com/ganlumomo/DynamicSemanticMapping.git
* git checkout -b demo2 origin/demo2
* mkdir build
* cd build
* cmake ..
* make

### Run:
./main input_pointcloud.txt

### Input Point Cloud Data Format:
* Position    Scene Flow    Semantic Label
* x y z delta_x delta_y delta_z i(1-n)

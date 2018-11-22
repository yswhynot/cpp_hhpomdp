# Hybrid Hierarchical POMDP
This is the C++ implementation for our paper  
**[Indoor Pursuit-Evasion with Hybrid Hierarchical Partially Observable Markov Decision Processes for Multi-Robot Systems](https://www.ri.cmu.edu/wp-content/uploads/2018/10/Yisha_Work.pdf)**  
[Sha Yi](https://yswhynot.github.io), [Changjoo Nam](https://sites.google.com/site/changjoonam/), [Katia Sycara](https://www.ri.cmu.edu/ri-faculty/katia-sycara/)

Please cite our work if you hope to use this in your research:
```
@conference{Yi2018Indoor,
author = {Sha Yi and Changjoo Nam and Katia Sycara},
title = {Indoor Pursuit-Evasion with Hybrid Hierarchical Partially Observable Markov Decision Processes for Multi-Robot Systems},
booktitle = {International Symposium on Distributed Autonomous Robotic Systems (DARS)},
year = {2018},
month = {October},
publisher = {Springer},
} 
```

## Setup
The core is written in C++. You might need `c++11`, `Eigen`, `Boost`, `CMake >= 2.8`. The command line parser is [cxxopts](https://github.com/jarro2783/cxxopts).  
The visualization is written in python. You might need `opencv`, `numpy`.

## Builds
To build the project:
```
mkdir bin
cd bin
cmake ..
make
```

## Demo
To run our demo, simply go to the binarys (if you follow the instruction, `cd bin/`), and
```
./experiment -e 1 -m 4 -n 2 -s
```
To visualize the output, go the `scripts/` directory, and
```
python show_proc.py 0 4 2
```

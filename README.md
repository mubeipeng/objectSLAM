# object_slam
This is the code for "SLAM with objects using a nonparametric pose graph". See bib reference below:

@INPROCEEDINGS{Mu_iros_2016,
	author={B. Mu and S. Y. Liu and L. Paull and J. Leonard and J. P. How},
	booktitle={2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	title={SLAM with objects using a nonparametric pose graph},
	year={2016},
	pages={4602-4609},
	keywords={SLAM (robots);graph theory;object detection;pose estimation;robot vision;sensor fusion;SLAM;data association;nonparametric pose graph;object detection;robotic application;simultaneous localization and mapping;Machine learning;Object detection;Proposals;Robustness;Simultaneous localization and mapping;Three-dimensional displays},
	doi={10.1109/IROS.2016.7759677},
	month={Oct},}

## iSAM library
Folder isam contains the modified isam library to optimize pose graphs. There are pre-compiled executable file isam is under the bin folder
To compile from source, following the commands on ubuntu:
cd isam
mkdir build && cd build && cmake ..
make
Fore more details about the library, refer to readme file under isam folder.

## Simulation
To generate simulated dataset, run generateSimData.m. Ground truth objects are randomly generated. Click in the figure to generate the ground truth trajectory, make sure there are enough loop closures. When finished, press enter button on the keyboard.

To run the algorithm and compared algorithms, run main_simulation.m

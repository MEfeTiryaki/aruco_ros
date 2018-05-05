aruco_ros
=========

Software package and ROS wrappers of the [Aruco][1] Augmented Reality marker detector library. More info can be found in the origin of the fork [aruco](https://github.com/pal-robotics/aruco_ros).

# Additions
## Multi_board application
The multi board detection currently detects the board with 6 marker as in [here](https://github.com/Sadetra/cri_efe/tree/master/cri_efe_aruco/urdf/multi_markers). 

### How to use
* Print your marker board with 6 markers of your choice
* Create your parameters.yaml like [this](https://github.com/Sadetra/cri_efe/blob/master/cri_efe_aruco/config/parameters.yaml). "board_id" is the aruco id of each board. "board_x/y" are the distance of the center of the each markers from the center of the multi_board. "color_r/g/b" are to visualize marker detection in rviz.
* Create a launch file like [this](https://github.com/Sadetra/cri_efe/blob/master/cri_efe_aruco/launch/multi.launch). "marker_size", "camera_name", "camera_frame" and 'marker_frame" frames are the important arguments here. Be aware of camera topics.
* When you launch, it detect markers and calculate the postion and orientation of the multi_marker by avaraging the position and orientation of the each marker
* To test markers on Gazebo simulations, the easiest way to generate markers in Gazebo is drawing them in urdf. You can find first 7 markers [here](https://github.com/Sadetra/cri_efe/tree/master/cri_efe_aruco/urdf/single_markers)

### Next steps and future work
* Orientation averaging
* Marker urdf generator
* Different multi_board schemes 


[1]: http://www.sciencedirect.com/science/article/pii/S0031320314000235 "Automatic generation and detection of highly reliable fiducial markers under occlusion by S. Garrido-Jurado and R. Muñoz-Salinas and F.J. Madrid-Cuevas and M.J. Marín-Jiménez 2014"

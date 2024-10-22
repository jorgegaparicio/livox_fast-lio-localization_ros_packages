# livox_fast-lio-localization_ros_packages
Different packages required to use fast-lio-localization with livox mid360 and mavros, ardupilot.
These packages are to be installed in the Ground Control Station (GCS).

## odom_to_global
Calculates the position of the drone refered to the map's system of reference, based on the Odometry information from the FAST LIO LOCALIZATION and the map origin.

## global_pose_to_flat_map
Takes the pose calculated by odom_to_global and publishes a /flat_map_pose topic with the required information for a 2d representation of the position of the drone.

## pcd_map_viewer
To use this package, a .pcd file must be included in catkin_ws/src/pcd_map_viewer/maps/scans_obstaculos.pcd
It represents the specified .pcd file in rviz.

## waypoint_publisher
Represents the current position of the drone, based on the information from the FAST LIO LOCALIZATION.
In order to publish a waypoint, a location should be clicked on the RViz map with the 2D Nav Goal tool

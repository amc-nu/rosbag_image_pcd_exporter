# Data Exporter

This simple node will export pointcloud data and image data to PCD and JPG files respectively.

The exported data will be saved under the ${HOME} directory of the user under the name `output_YYYYMMDDHHmmSS`.
 
 The data PCD and JPG data will be stored in subdirectories as shown below:
 
 ```
 output_YYYYmmdd_HHMMSS/
 ├── image
 └── pointcloud
 ```
 
 The files under `image` and `pointcloud` will be named in numerical sequence base 0, padded with upto 6 zeros (i.e. 000001.pcd or 000001.jpg).
 
 ## Topic Subscription
 |Param|Default Value|Description|
 |---|---|---|
 |`image_src`|`image_raw`|image data|
 |`points_src`|`points_raw`|point cloud data|
 |`sync`|`False`|Whether or not to require synchronized topics. If set to `True`, the timestamps from both the image and cloud must match. When `False`, the data will be processed as it arrives.|
 
 ## How to run
 
 1. Compile using `catkin_make` under a valid catkin workspace.
 2. Source the terminal in the workspace `source devel/setup.bash`.
 2. Execute `rosrun data_exporter image_pointcloud_exporter`.
 
 
 
 
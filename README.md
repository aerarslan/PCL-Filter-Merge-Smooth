# Point Cloud Library Filter, Merge, Smooth
A sub app for body scanner project. This app is used to filter and smooth a point cloud and merge it with a given skeleton coordinate map.

## [Find the Body Scanner app from here](https://github.com/aerarslan/BodyScanner-Kinect)

•	Convert Button: gets a pcd file then saves it in .pcd format to the target location.
o	Usage: pcl_converter_release.exe -c C:\Desktop\myObject.stl C:\Desktop\myPCD.pcd
•	View Button: visualizes a .pcd file
o	Usage: pcl_viewer_release.exe C:\Desktop\myPCD.pcd
•	Filter Button: filters the given .pcd file then saves it to the target location.
o	Usage: SubPrograms.exe f C:\Desktop\myPCD.pcd C:\Desktop\myPCD_filtered.pcd
•	Smooth Button: smoothes the given .pcd file then saves it to the target location.
o	Usage: SubPrograms.exe s C:\Desktop\myPCD.pcd C:\Desktop\myPCD_smoothed.pcd
•	Merge Button: merges the given .pcd and .csv files the saves it to the target location.
o	Usage: SubPrograms.exe m C:\Desktop\myPCD.pcd C:\Desktop\mySkeletonCoordinates.csv C:\Desktop\merged.csv


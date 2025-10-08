# 3D Plot Clip
This repository contains code for cropping plots from the geocorrected 3D point clouds. Geocorrected 3D point clouds from scanner measurements are cropped to geoJSON shape files, then merged.

## Inputs
Geocorrected 3D point clouds that have undergone pre- and post-processing.

## Outputs
Subdirectories named after agricultural plots. Each subdirectory contains a single 3D point cloud that consitute an individual plot.

## Arguments and Flags
* **Required Arguments:**
  * **Input directory containing point clouds:** '-i', '--input'
  * **Output directory:** '-o', '--output'
  * **geoJSON file defining plot:** '-g', '--geojson'
  * **Date of data collection:** '-d', '--date'

# 3D Plotclip
This repository contains code for cropping plots from the 3D point clouds. 3D point clouds from scanner measurements are first geocorrected. Geocorrected files are merged and full-scale and downsampled merged point clouds are output. Geocorrected point clouds are cropped to the geoJSON shapefile to result in a directory containing partial cropped PLY files for each agricultural plot. Partial crops are then merged to result in full crops for each plot.

## Inputs
3D point clouds that have undergone preprocessing (i.e., level 1 3D outputs)

Transformation JSON output from 3D Landmark selection.

Season GeoJSON defining plot boundaries.

## Outputs
Subdirectories named after agricultural plots. Each subdirectory contains a single 3D point cloud that consitute an individual plot.

## Arguments and Flags
* **Required Arguments:**
  * **Input directory containing point clouds:** '-i', '--input'
  * **Output directory:** '-o', '--output'
  * **Transformation (from landmark selection):** '-t', '--transformation'
  * **geoJSON file defining plot:** '-g', '--geojson'
  * **Date of data collection:** '-d', '--date'
  * **Maximum number of CPUs to use in geocorrection multiprocessing (default: 4):** '--cores_geo'
  * **Maximum number of CPUs to use in cropping multiprocessing (default: 4):** '--cores_crop'
  * **Enable dynamic core allocation based on available resources:** '--dynamiccores'
    
* **Optional Arguments:** 
  * **Maximum number of points in millions for additional downsampled output point cloud (default: 1):** '-p', '--points'
  * **Disable geocorrection:** '--disablegeo'
  * **Disable saving of merged point cloud:** '--disablepcd'
  * **Disable saving of only full-scale merged point cloud:** '--disablefpcd'
  * **Disable cropping of merged point cloud:** '--disablecrop'

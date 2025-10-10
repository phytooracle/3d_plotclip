import open3d as o3d
import os
import json
import numpy as np
import open3d as o3d
from pyproj import CRS, transform, Transformer
    
proj_latlon = "EPSG:4326" #CRS.from_epsg(4326) # WGS 84 -- WGS84 - World Geodetic System 1984, used in GPS
proj_utm = "EPSG:32612" #CRS.from_epsg(32612) # WGS 84 / UTM zone 12N
transformer_to_latlon = Transformer.from_crs(proj_utm, proj_latlon, always_xy=True)
transformer_to_utm = Transformer.from_crs(proj_latlon, proj_utm, always_xy=True)

def utm_to_latlon(easting, northing):
    """
    Convert UTM coordinates to geographic coordinates
    """
    try:
        lon, lat = transformer_to_latlon.transform(easting, northing)
        return lon, lat
    except Exception as e:
        print(f"Error converting UTM to lat/lon: {e}")
        return None

def latlon_to_utm(lon, lat):
    """
    Convert geographic coordinates to UTM coordinates
    """
    try:
        easting, northing = transformer_to_utm.transform(lon, lat)
        return easting, northing
    except Exception as e:
        print(f"Error converting lat/lon to UTM: {e}")
        return None

def transform_pcd(pcd, T):
    """
    Adjust XY using affine transformation. Convert Z from millimeters to meters. Align bounding box
    center and dimensions. Return transformed point cloud in meters.
    """
    points_pos = np.array(pcd.points)
    z_points_pos = np.copy(points_pos[:,2])
    points_pos[:,2] = 1
    transformed_points_pos = np.matmul(T,points_pos.T).T
    transformed_points_pos = transformed_points_pos[:,:2]
    transformed_points_pos = np.hstack((transformed_points_pos, np.expand_dims(z_points_pos*0.001,axis=-1)))
    transformed_pcd_pos = o3d.geometry.PointCloud() 
    transformed_pcd_pos.points = o3d.utility.Vector3dVector(transformed_points_pos)
    
    # Target bounding box
    min_target = transformed_pcd_pos.get_min_bound()
    max_target = transformed_pcd_pos.get_max_bound()
    center_target = (min_target + max_target) / 2.0
    del min_target, max_target
    
    del transformed_pcd_pos, points_pos, z_points_pos, transformed_points_pos, 

    points = np.asarray(pcd.points)

    # Step 1: Scale XY up by 1000 before applying T
    xy_scaled_up = points[:, :2] * 1000  # Pretend mm → exaggerated mm

    # Step 2: Add homogeneous coordinate
    ones = np.ones((xy_scaled_up.shape[0], 1))
    xy_hom = np.hstack((xy_scaled_up, ones))

    # Step 3: Apply transformation
    transformed_xy = (T @ xy_hom.T).T  # Shape: (N, 2)

    # Step 4: Normalize back by dividing by 1000 (undo exaggeration)
    normalized_xy = transformed_xy / 1000.0

    # Step 5: Convert Z to meters
    z_m = points[:, 2] * 0.001

    # Step 6: Combine normalized XY with scaled Z
    transformed_points = np.hstack((normalized_xy, z_m.reshape(-1, 1)))

    # Step 7: Create new point cloud
    transformed_pcd = o3d.geometry.PointCloud()
    transformed_pcd.points = o3d.utility.Vector3dVector(transformed_points)
    del transformed_points, transformed_xy, ones, xy_hom, xy_scaled_up, z_m, 
    
    # Current bounding box
    min_current = transformed_pcd.get_min_bound()
    max_current = transformed_pcd.get_max_bound()
    center_current = (min_current + max_current) / 2.0
    del min_current, max_current
    
    # Compute scale and translation
    offset = center_target - center_current
    print("Center target: ", center_target)
    print("Center current: ", center_current)
    print("Translation offset: ", offset)
    transformed_pcd.translate(offset)
    del offset, center_target, center_current

    return transformed_pcd

def postprocess_single_pass(path, outpath, folder, transformation, current_date):
    """
    Load point cloud for single scan pass. Apply geocorrection using transformation matrix.
    Compute bounding box stats, color point cloud, and save geocorrected result.
    """
    # Load transformation matrix
    with open(transformation, 'r') as f:
        tr = json.load(f)
    T = np.array(tr['transformation'])

    # Get paths
    path_dict = get_path_dict(path, outpath, folder)

    # Load and transform point cloud
    pcd = load_pcd(path_dict['aligned_merged_path'])

    # Apply original transformation
    transformed_pcd = transform_pcd(pcd, T)

    # Compute bounding box center and dimensions
    min_bound_pcd = pcd.get_min_bound()
    max_bound_pcd = pcd.get_max_bound()
    center_pcd = (min_bound_pcd + max_bound_pcd) / 2.0
    dimensions_pcd = max_bound_pcd - min_bound_pcd
    del pcd, min_bound_pcd, max_bound_pcd
    
    min_bound = transformed_pcd.get_min_bound()
    max_bound = transformed_pcd.get_max_bound()
    center = (min_bound + max_bound) / 2.0
    dimensions = max_bound - min_bound
    del min_bound, max_bound

    # Format and print debug information
    center_formatted_pcd = [f"{c:.1f}" for c in center_pcd]
    dimensions_formatted_pcd = [f"{d:.1f}" for d in dimensions_pcd]
    del center_pcd, dimensions_pcd
    print(f"Original point cloud bounding box center: {center_formatted_pcd}")
    print(f"Original Bounding box dimensions (width, height, depth): {dimensions_formatted_pcd}")
    del center_formatted_pcd, dimensions_formatted_pcd
    
    center_formatted = [f"{c:.1f}" for c in center]
    dimensions_formatted = [f"{d:.1f}" for d in dimensions]
    del center, dimensions
    print(f"Corrected point cloud bounding box center: {center_formatted}")
    print(f"Corrected Bounding box dimensions (width, height, depth): {dimensions_formatted}")
    del center_formatted, dimensions_formatted

    # Paint and save
    painted_pcd = paint_pcd(transformed_pcd)
    save_pcd(painted_pcd, path_dict['geocorrected_merged_path'])
    print(f"Saving geocorrected point cloud to {path_dict['geocorrected_merged_path']}\n")
    
def save_pcd(pcd,path):
    """
    Write point cloud to disk in binary PLY format using Open3D
    """
    o3d.io.write_point_cloud(path, pcd, write_ascii=False)

def load_pcd(path):
    """
    Read point cloud from disk in PLY format using Open3D
    """
    pcd = o3d.io.read_point_cloud(path,format="ply")
    return pcd

def paint_pcd(pcd):
    """
    Apply color gradient to a point cloud based on z-axis height. 
    High points yellow. Low points red. Return colored point cloud.
    """
    color1 = (255/255, 255/255, 0)
    color2 = (170/255, 0, 0)

    pcd.paint_uniform_color([1,1,1])
    points = np.array(pcd.points)

    mins = np.min(points,axis=0)
    maxs = np.max(points,axis=0)

    ratios = (points[:,2]-mins[2])/(maxs[2]-mins[2])
    ratios = np.vstack((ratios,ratios,ratios)).T
    colors = np.array((ratios*color1+(1-ratios)*color2))
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    del points, mins, maxs, ratios, colors, color1, color2
    return pcd

def get_path_dict(path,outpath,folder_name):
    """
    Generate and return dictionary of file paths for input merged point cloud, 
    output geocorrected point cloud, pass ID, and folder name.
    """
    if path[-1] == '/':
        path = path[:-1]

    merged_path = os.path.join(path,"merged",folder_name)
    # Check if the merged_path directory is not empty
    merged_files = os.listdir(merged_path)
    if not merged_files:
        raise FileNotFoundError(f"No files found in merged path: {merged_path}")    
    
    pass_id = os.listdir(os.path.join(path,"merged",folder_name))[0].split('/')[-1].split('_')[0]

    merged_outpath = os.path.join(outpath,"merged_geocorrected",folder_name)

    os.makedirs(merged_outpath,exist_ok=True)

    aligned_merged_path = os.path.join(merged_path,f"{pass_id}__Top-heading-merged.ply")
    geocorrected_merged_path = os.path.join(merged_outpath,f"{pass_id}__Top-heading-merged.ply")

    path_dict = {}
    path_dict['aligned_merged_path'] = aligned_merged_path
    path_dict['geocorrected_merged_path'] = geocorrected_merged_path
    path_dict['pass_id'] = pass_id
    path_dict['folder_name'] = folder_name

    return path_dict
   
def load_plots(geojson_path):
    """
    Parse GeoJSON file to extract plot definitions. Return dictionary of plot IDs with corner and center coordinates.
    """
    plots = {}

    with open(geojson_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    for feature in data['features']:
        plot_id = feature['properties'].get('ID')
        geometry = feature['geometry']

        if geometry['type'] == 'MultiPolygon':
            # Get the first polygon's exterior ring
            polygon = geometry['coordinates'][0]
            if not polygon or len(polygon[0]) < 4:
                continue  # Not enough points

            ring = polygon[0]  # exterior ring
            UL = ring[0]
            UR = ring[1]
            LR = ring[2]
            LL = ring[3]

            xs = [pt[0] for pt in ring[:4]]
            ys = [pt[1] for pt in ring[:4]]
            C = [sum(xs)/4, sum(ys)/4]

            plots[plot_id] = {
                'UL': UL,
                'UR': UR,
                'LL': LL,
                'LR': LR,
                'C': C
            }
    
    return plots
    
def crop_and_save_plots(pcd, plots, outpath, transformation_json_path, force_crop=False):
    """
    Iterate through plots and crops corresponding regions from merged point cloud. 
    Converts plot coordinates to UTM, crops point cloud using bounding polygons, 
    and saves cropped plots to disk.
    """
    boundaries = get_boundings_pcd(pcd, tolatlon=True)

    # Load transformation matrix
    with open(transformation_json_path, 'r') as f:
        T = np.array(json.load(f)["transformation"])

    for plot_id, coord in plots.items():
        lon, lat = coord['C']
        print(f"Plot {plot_id} center: {lon}, {lat}")
        print(f"Bounding box: {boundaries}")

        should_crop = force_crop or check_point_in_boundaries(lon, lat, boundaries)
        if should_crop:
            print(f"Cropping and saving plot {plot_id}")

            # Step 1: Convert laton to utm
            UL = latlon_to_utm(coord['UL'][0], coord['UL'][1])
            UR = latlon_to_utm(coord['UR'][0], coord['UR'][1])
            LL = latlon_to_utm(coord['LL'][0], coord['LL'][1])
            LR = latlon_to_utm(coord['LR'][0], coord['LR'][1])

            # Step 2: Crop
            plot_pcd = crop_single_plot((UL, UR, LL, LR, pcd))

            print("Checking if empty...")
            if not plot_pcd.is_empty():
                outpath = outpath.encode('ascii', 'ignore').decode('ascii').strip()
                plot_id_clean = plot_id.encode('ascii', 'ignore').decode('ascii').strip()
                painted_pcd = paint_pcd(plot_pcd)
                del plot_pcd
                save_path = os.path.join(outpath, f"{plot_id_clean}.ply")
                print("Saving to ", save_path)
                save_pcd(painted_pcd, save_path)
                print(f"Saved {save_path}\n")
            else:
                print(f"plot_pcd is empty for plot {plot_id}!")
                print("Cropping polygon might be outside bounds.\n")
        else:
            print(f"Skipping plot {plot_id}: outside bounding box\n")
            
def get_boundings_pcd(pcd,tolatlon=False):
    """
    Compute bounding box of point cloud. Returns min and max coordinates. 
    Optionally converts bounds to geographic coordinates.
    """
    mins = np.min(np.array(pcd.points),axis=0)
    maxs = np.max(np.array(pcd.points),axis=0)

    if not tolatlon:
        return {"mins":list(mins),"maxs":list(maxs)}
    else:
        new_mins = utm_to_latlon(mins[0],mins[1])
        new_maxs = utm_to_latlon(maxs[0],maxs[1])
        return {"mins":list(new_mins),"maxs":list(new_maxs)}
        
def crop_single_plot(args):
    """
    Crop point cloud using polygon defined by four UTM coordinates. 
    Creates bounding polygon, crops with Open3D, then returns cropped point cloud.
    """
    print("Cropping single plot...")
    UL = args[0]
    UR = args[1]
    LL = args[2]
    LR = args[3]
    pcd = args[4]
    print("Expecting UTM coordinates for cropping polygon")
    print("UL:", UL)
    print("UR:", UR)
    print("LL:", LL)
    print("LR:", LR)
    width = abs(UR[0] - UL[0])
    height = abs(UL[1] - LL[1])
    r_x = width * 0
    r_y = height * 0
    max_x, max_y, max_z = pcd.get_max_bound()
    min_x, min_y, min_z = pcd.get_min_bound()
    bounding_polygon = np.array([
        [UL[0] - r_x, UL[1] - r_y, 0],
        [UR[0] + r_x, UR[1] - r_y, 0],
        [LR[0] + r_x, LR[1] + r_y, 0],
        [LL[0] - r_x, LL[1] + r_y, 0]
    ]).astype('float64')
    print("Bounding polygon (UTM):", bounding_polygon)
    print("Point cloud bounds:", min_x, min_y, min_z, max_x, max_y, max_z)
    vol = o3d.visualization.SelectionPolygonVolume()
    vol.orthogonal_axis = "Z"
    vol.axis_max = max_z
    vol.axis_min = min_z
    vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
    plot = vol.crop_point_cloud(pcd)
    print("Finished cropping single plot")
    return plot

def check_point_in_boundaries(lon,lat,boundaries):
    """
    Checks if a given geographic point lies within specified bounding box limits.
    """
    return lon>boundaries['mins'][0] and lon<boundaries['maxs'][0] and lat>boundaries['mins'][1] and lat<boundaries['maxs'][1]

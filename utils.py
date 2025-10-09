import open3d as o3d
import os
import json
import numpy as np
import open3d as o3d
from pyproj import Proj,transform

proj_4326 = Proj('epsg:4326') # WGS84 lat/lon
proj_utm = Proj('epsg:32612') # UTM Zone 12N for Arizona


def get_path_dict_cropping(path,outpath,folder_name):
    if path[-1] == '/':
        path = path[:-1]
    
    folder_name = folder_name.replace('/','')

    merged_path = os.path.join(path,"merged",folder_name)
    # Check if the merged_path directory is not empty
    merged_files = os.listdir(merged_path)
    if not merged_files:
        raise FileNotFoundError(f"No files found in merged path: {merged_path}")
    
    pass_id = os.listdir(merged_path)[0].split('/')[-1].split('_')[0]
    geocorrected_merged_path = os.path.join(merged_path,f"{pass_id}__Top-heading-merged.ply")

    merged_outpath = os.path.join(outpath,"merged")
    if not os.path.exists(merged_outpath):
        os.makedirs(merged_outpath)

    path_dict_cropping = {}
    
    path_dict_cropping['geocorrected_merged_path'] = geocorrected_merged_path
    path_dict_cropping['cropped_merged_path'] = merged_outpath
    path_dict_cropping['pass_id'] = pass_id
    path_dict_cropping['folder_name'] = folder_name

    return path_dict_cropping

def crop_plots(path,outpath,folder,plotpath,current_date):
    path_dict_cropping = get_path_dict_cropping(path,outpath,folder)
    plots = load_plots(plotpath,current_date)

    print(f":: Beginning processing {path_dict_cropping['folder_name']}.")
    
    pcd_path = path_dict_cropping['geocorrected_merged_path']
    pcd = o3d.io.read_point_cloud(pcd_path)
    cropped_plots = crop_all_plots(pcd,plots)
    save_cropped_plots(cropped_plots,path_dict_cropping['cropped_merged_path'],path_dict_cropping['folder_name'])

def load_pcd(path):
    pcd = o3d.io.read_point_cloud(path,format="ply")
    return pcd

def latlon_to_utm(lon, lat):
    return proj_utm(lon, lat)

def utm_to_latlon(easting, northing):
    return proj_utm(easting, northing, inverse=True)

def get_boundings_pcd(pcd,tolatlon=False):
    mins = np.min(np.array(pcd.points),axis=0)
    maxs = np.max(np.array(pcd.points),axis=0)

    if not tolatlon:
        return {"mins":list(mins),"maxs":list(maxs)}
    else:
        new_mins = utm_to_latlon(mins[0],mins[1])
        new_maxs = utm_to_latlon(maxs[0],maxs[1])
        return {"mins":list(new_mins),"maxs":list(new_maxs)}

def load_plots(geojson_path):
    """
    Loads plot data from a GeoJSON file.

    Parameters:
    - geojson_path: Path to the GeoJSON file.

    Returns:
    - plots: A dictionary where each key is a plot ID and the value is a dictionary
             containing corner coordinates and center point.
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

def crop_single_plot(args):
    print("Cropping single plot...")
    UL = args[0]
    UR = args[1]
    LL = args[2]
    LR = args[3]
    pcd = args[4]

    width = abs(UR[0] - UL[0])
    height = abs(UL[1] - LL[1])

    r_x = width * 1.0
    r_y = height * 1.0

    max_x, max_y, max_z = pcd.get_max_bound()
    min_x, min_y, min_z = pcd.get_min_bound()

    bounding_polygon = np.array([
        [max(min_x, UL[0] - r_x), max(min_y, UL[1] - r_y), 0],
        [min(max_x, UR[0] + r_x), max(min_y, UR[1] - r_y), 0],
        [min(max_x, LR[0] + r_x), min(max_y, LR[1] + r_y), 0],
        [max(min_x, LL[0] - r_x), min(max_y, LL[1] + r_y), 0]
    ]).astype('float64')

    vol = o3d.visualization.SelectionPolygonVolume()
    vol.orthogonal_axis = "Z"
    vol.axis_max = max_z
    vol.axis_min = min_z
    vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
    
    print("Cropping bounds:")
    print("UL:", UL)
    print("UR:", UR)
    print("LL:", LL)
    print("LR:", LR)
    print("Bounding polygon:", bounding_polygon)
    print("Point cloud bounds:", min_x, min_y, min_z, max_x, max_y, max_z)

    plot = vol.crop_point_cloud(pcd)
    print("Finished cropping single plot")
    
    return plot

def check_point_in_boundaries(lon,lat,boundaries):
    return lon>boundaries['mins'][0] and lon<boundaries['maxs'][0] and lat>boundaries['mins'][1] and lat<boundaries['maxs'][1]

def crop_all_plots(pcd, plots, force_crop=False):
    cropped_plots = {}
    boundaries = get_boundings_pcd(pcd, tolatlon=True)

    for plot_id, coord in plots.items():
        lon, lat = coord['C']
        print(f"Plot {plot_id} center: {lon}, {lat}")
        print(f"Bounding box: {boundaries}")

        should_crop = force_crop or check_point_in_boundaries(lon, lat, boundaries)
        if should_crop:
            print(f"Cropping plot {plot_id}")
            UL = latlon_to_utm(coord['UL'][0], coord['UL'][1])
            UR = latlon_to_utm(coord['UR'][0], coord['UR'][1])
            LL = latlon_to_utm(coord['LL'][0], coord['LL'][1])
            LR = latlon_to_utm(coord['LR'][0], coord['LR'][1])
            plot_pcd = crop_single_plot((UL, UR, LL, LR, pcd))
            cropped_plots[plot_id] = plot_pcd
        else:
            print(f"Skipping plot {plot_id}: outside bounding box")

    return cropped_plots

def paint_pcd(pcd):
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
    return pcd

def save_cropped_plots(cropped_plots, outpath, identifier):
    for plot_id, pcd in cropped_plots.items():
        if not pcd.is_empty():
            outpath = outpath.encode('ascii', 'ignore').decode('ascii').strip()
            plot_id = plot_id.encode('ascii', 'ignore').decode('ascii').strip()
            plot_path = os.path.join(outpath, plot_id)
            
            if not os.path.exists(plot_path):
                os.mkdir(plot_path)
            
            painted_pcd = paint_pcd(pcd)
            save_path = os.path.join(plot_path, f"{identifier}_cropped.ply")
            o3d.io.write_point_cloud(save_path, painted_pcd)
            print("Saved ", save_path)

def crop_and_save_plots(pcd, plots, outpath, identifier, force_crop=False):
    boundaries = get_boundings_pcd(pcd, tolatlon=True)

    for plot_id, coord in plots.items():
        lon, lat = coord['C']
        print(f"Plot {plot_id} center: {lon}, {lat}")
        print(f"Bounding box: {boundaries}")

        should_crop = force_crop or check_point_in_boundaries(lon, lat, boundaries)
        if should_crop:
            print(f"Cropping and saving plot {plot_id}")
            UL = latlon_to_utm(coord['UL'][0], coord['UL'][1])
            UR = latlon_to_utm(coord['UR'][0], coord['UR'][1])
            LL = latlon_to_utm(coord['LL'][0], coord['LL'][1])
            LR = latlon_to_utm(coord['LR'][0], coord['LR'][1])
            plot_pcd = crop_single_plot((UL, UR, LL, LR, pcd))

            print("Checking if empty...")
            if not plot_pcd.is_empty():
                outpath = outpath.encode('ascii', 'ignore').decode('ascii').strip()
                plot_id_clean = plot_id.encode('ascii', 'ignore').decode('ascii').strip()
                plot_path = os.path.join(outpath, plot_id_clean)

                if not os.path.exists(plot_path):
                    os.mkdir(plot_path)

                painted_pcd = paint_pcd(plot_pcd)
                save_path = os.path.join(plot_path, f"{identifier}_cropped.ply")
                print("Saving to ", save_path)
                o3d.io.write_point_cloud(save_path, painted_pcd)
                print("Saved\n", save_path)
            else:
                print("plot_pcd is empty!\n")    
        else:
            print(f"Skipping plot {plot_id}: outside bounding box\n")

def pairwise_registration(source, target,max_correspondence_distance_coarse, max_correspondence_distance_fine):
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp

def full_registration(pcds, max_correspondence_distance_coarse, max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))

    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id],max_correspondence_distance_coarse,max_correspondence_distance_fine)
          
            odometry = np.dot(transformation_icp, odometry)
            pose_graph.nodes.append(
                o3d.pipelines.registration.PoseGraphNode(
                    np.linalg.inv(odometry)))
            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                            target_id,
                                                            transformation_icp,
                                                            information_icp,
                                                            uncertain=False))

    return pose_graph

def full_register_merge_pcds(pcds):
    voxel_size = 1e-3
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(pcds,
                                    max_correspondence_distance_coarse,
                                    max_correspondence_distance_fine)

    option = o3d.pipelines.registration.GlobalOptimizationOption(
    max_correspondence_distance=max_correspondence_distance_fine,
    edge_prune_threshold=0.25,
    reference_node=0)
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option)

    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]
    
    return pcd_combined
    
def load_plot_pcds(plot_folder):
    """
    Load all .ply files from a plot folder.
    """
    pcds = []
    for filename in os.listdir(plot_folder):
        if filename.endswith("_cropped.ply"):
            file_path = os.path.join(plot_folder, filename)
            pcd = o3d.io.read_point_cloud(file_path)
            if not pcd.is_empty():
                pcds.append(pcd)
    return pcds

def merge_plot_point_clouds(base_path, output_path):
    """
    For each plot folder in base_path, merge all cropped point clouds and save the result.
    """
    for plot_id in os.listdir(base_path):
        plot_folder = os.path.join(base_path, plot_id)
        if os.path.isdir(plot_folder):
            print(f"Processing plot: {plot_id}")
            pcds = load_plot_pcds(plot_folder)
            if len(pcds) > 1:
                merged_pcd = full_register_merge_pcds(pcds)
            elif len(pcds) == 1:
                merged_pcd = pcds[0]
            else:
                continue  # Skip empty folders

            # Save merged point cloud
            save_path = os.path.join(output_path, f"{plot_id}_merged.ply")
            o3d.io.write_point_cloud(save_path, merged_pcd)
            print(f"Saved merged point cloud to: {save_path}")
                  
def get_path_dict(path,outpath,folder_name):
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
    
'''def transform_pcd(pcd,T):
    points = np.array(pcd.points)
    z_points = np.copy(points[:,2])
    points[:,2] = 1
    transformed_points = np.matmul(T,points.T).T
    transformed_points = transformed_points[:,:2]
    transformed_points = np.hstack((transformed_points, np.expand_dims(z_points*0.001,axis=-1)))
    transformed_pcd = o3d.geometry.PointCloud() 
    transformed_pcd.points = o3d.utility.Vector3dVector(transformed_points)
    return transformed_pcd'''
    
def transform_pcd(pcd, T, xy_scale=1.0, z_scale=1.0):
    points = np.array(pcd.points)
    xy = points[:, :2] * xy_scale
    ones = np.ones((xy.shape[0], 1))
    xy_hom = np.hstack((xy, ones))
    transformed_xy = (T @ xy_hom.T).T
    transformed_points = np.hstack((transformed_xy, points[:, 2:3] * z_scale))

    transformed_pcd = o3d.geometry.PointCloud()
    transformed_pcd.points = o3d.utility.Vector3dVector(transformed_points)

    return transformed_pcd

def save_pcd(pcd,path):
    o3d.io.write_point_cloud(path, pcd)

def postprocess_single_pass(path,outpath,folder,transformation,current_date):
    with open(transformation,'r') as f:
        tr = json.load(f)

    T = np.array(tr['transformation'])

    path_dict = get_path_dict(path,outpath,folder)

    pcd = load_pcd(path_dict['aligned_merged_path'])
    transformed_pcd = transform_pcd(pcd,T, xy_scale=1.0, z_scale=0.001)
    painted_pcd = paint_pcd(transformed_pcd)
    save_pcd(painted_pcd,path_dict['geocorrected_merged_path'])
    print(f"Saving geocorrected point cloud to {path_dict['geocorrected_merged_path']}")
            

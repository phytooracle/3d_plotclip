import argparse
from utils import *
import tempfile
import shutil
from multiprocessing import Pool, get_context
import time
import sys
import resource

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)

def get_args():
    
    parser = argparse.ArgumentParser(
        description='3D plot clipping pipeline',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('-i',
                        '--input',
                        help='Path to the directory that contains all different folders of preprocessing/alignment (west, east, merged, ...).',
                        metavar='str',
                        type=str,
                        required=True)

    parser.add_argument('-o',
                        '--output',
                        help='The output directory to which the results will be saved.',
                        metavar='str',
                        type=str,
                        required=True)

    parser.add_argument('-t',
                        '--transformation',
                        help='The transformation json file',
                        metavar='transformation',
                        type=str,
                        required=True)

    parser.add_argument('-g',
                        '--geojson',
                        help='The geoJSON defining plot',
                        metavar='str',
                        type=str,
                        required=True)

    parser.add_argument('-d',
                        '--date',
                        help='The scan date string in format YYYY-MM-DD',
                        metavar='str',
                        type=str,
                        required=True)
                        
    parser.add_argument('-p',
                        '--points',
                        help='Maximum number of points in millions for additional downsampled output point cloud (integer).',
                        type=int,
                        default=1)

    parser.add_argument('--disablegeo',
                        help='disables geocorrection',
                        action='store_true')

    parser.add_argument('--disablepcd',
                        help='disables saving of merged pcd',
                        action='store_true')

    parser.add_argument('--disablefpcd',
                        help='disables saving of only full-scale merged pcd',
                        action='store_true')

    parser.add_argument('--disablecrop',
                        help='disables cropping of merged pcd',
                        action='store_true')
                        
    parser.add_argument('--cores_geo',
                        help='Maximum number of cpus to use in geocorrection multiprocessing.',
                        type=int,
                        default=4)

    parser.add_argument('--cores_crop',
                        help='Maximum number of cpus to use in cropping multiprocessing.',
                        type=int,
                        default=4)                        
                        
    return parser.parse_args()



def main():
    args = get_args()
    if not os.path.isdir(args.output):
        os.makedirs(args.output)

    # Step 1: Geocorrect all passes
    print("Geocorrection started\n", flush=True)
    start_time0 = time.perf_counter()
    if not args.disablegeo:
        folder_names = [
            f for f in os.listdir(os.path.join(args.input, "merged"))
            if os.path.isdir(os.path.join(args.input, "merged", f))
        ]

        args_list = [
            (folder, args.input, args.input, args.transformation, args.date)
            for folder in folder_names
        ]
        core_count = min(args.cores_geo, len(folder_names))
        with Pool(processes=core_count) as pool:
            pool.map(process_folder, args_list)

        end_time = time.perf_counter()
        elapsed_time_geo = (end_time - start_time0) / 60
        print(f"Elapsed time for geocorrection: {elapsed_time_geo:.4f} minutes", flush=True)

    # Step 2: Merge all geocorrected point clouds
    print("Merging started\n", flush=True)
    start_time = time.perf_counter()
    geocorrected_dir = os.path.join(args.input, "merged_geocorrected")
    merged_pcd = o3d.geometry.PointCloud()
    
    pass_ids = [
        d for d in os.listdir(geocorrected_dir)
        if os.path.isdir(os.path.join(geocorrected_dir, d))
    ]

    for idx, pass_id in enumerate(pass_ids, start=1):
        start_time_indpass = time.perf_counter()
        pass_dir = os.path.join(geocorrected_dir, pass_id)
        for filename in os.listdir(pass_dir):
            if filename.endswith(".ply"):
                pcd_path = os.path.join(pass_dir, filename)
                print(f"[{idx}/{len(pass_ids)}] Reading in {pcd_path}", flush=True)
                pcd = o3d.io.read_point_cloud(pcd_path)
                if not pcd.is_empty():
                    merged_pcd += pcd
                    del pcd
        end_time_indpass = time.perf_counter()
        elapsed_indpass = end_time_indpass - start_time_indpass
        print(f"[{idx}/{len(pass_ids)}] Finished processing {pass_id} in {elapsed_indpass:.2f} seconds\n", flush=True)
        
    print("Merging complete\n", flush=True)
    log_memory_usage("After merging point clouds")
    end_time = time.perf_counter()
    elapsed_time_merge = (end_time - start_time) / 60
    print(f"Elapsed time for merging: {elapsed_time_merge:.4f} minutes", flush=True)
    if not args.disablepcd:
        if args.points:
            start_time = time.perf_counter()
            max_points = args.points * 1_000_000
            current_points = len(merged_pcd.points)
            if current_points > max_points:
                print(f"Downsampling merged point cloud from {current_points} to {max_points} points...", flush=True)
                merged_pcd_output = merged_pcd.random_down_sample(max_points / current_points)
                merged_pcd_output_outpath = os.path.join(args.output, args.date + "_" + str(args.points) + "milMax_merged_geocorrected.ply")
                save_pcd(merged_pcd_output, merged_pcd_output_outpath)
                del merged_pcd_output
                if not args.disablefpcd:
                    merged_pcd_outpath = os.path.join(args.output, args.date + "_full" + "_merged_geocorrected.ply")
                    save_pcd(merged_pcd, merged_pcd_outpath)
            else:
                merged_pcd_outpath = os.path.join(args.output, args.date + "_full" + "_merged_geocorrected.ply")
                save_pcd(merged_pcd, merged_pcd_outpath)
            end_time = time.perf_counter()
            elapsed_time_pcd = (end_time - start_time) / 60
            print(f"Elapsed time for outputting point clouds: {elapsed_time_pcd:.4f} minutes", flush=True)
        
    
    if not args.disablecrop:
        # Step 3: Load plot definitions
        print("\nLoading plot definitions...", flush=True)
        start_time = time.perf_counter()
        plots = load_plots(args.geojson)
        print(f"[DEBUG] Number of plots loaded: {len(plots)}", flush=True)
        print("Plots loaded\n", flush=True)
        end_time = time.perf_counter()
        elapsed_time_plots = (end_time - start_time) / 60
        print(f"Elapsed time for loading plots: {elapsed_time_plots:.4f} minutes", flush=True)

        # Step 4: Crop plots from the transformed cloud
        print("Cropping plots from merged point cloud...", flush=True)
        start_time = time.perf_counter()
        
        # Step 4a: Set up memory-mapped file
        print("setting up memory-mapped file", flush=True)
        start_time = time.perf_counter()
        points = np.asarray(merged_pcd.points)
        dtype = points.dtype
        shape = points.shape
        try:  
            temp_dir = tempfile.gettempdir()
            memmap_path = os.path.join(temp_dir, "points_memmap.dat")
            memmap_array = np.memmap(memmap_path, dtype=dtype, mode='w+', shape=shape)
            memmap_array[:] = points[:]
        except Exception as e:
            print(f"[ERROR] Failed memory map: {e}", flush=True)
            sys.exit(1)
        log_memory_usage("After memory-mapping")
        end_time = time.perf_counter()
        elapsed_time_memmap = (end_time - start_time) / 60
        print(f"Elapsed time for memory mapping: {elapsed_time_memmap:.4f} minutes", flush=True)

        # Step 4b: Prepare arguments
        print("preparing args", flush=True)
        crop_outpath = os.path.join(args.output, "plotclip_out")
        os.makedirs(crop_outpath, exist_ok=True)
        
        print("getting boundaries", flush=True)    
        start_time = time.perf_counter()  
        bbox = merged_pcd.get_axis_aligned_bounding_box()
        mins = bbox.get_min_bound()
        maxs = bbox.get_max_bound()
        new_mins = utm_to_latlon(mins[0], mins[1])
        new_maxs = utm_to_latlon(maxs[0], maxs[1])
        boundaries = {"mins": list(new_mins), "maxs": list(new_maxs)}
        end_time = time.perf_counter()
        elapsed_time_bounds = (end_time - start_time) / 60
        print(f"Elapsed time for getting boundaries: {elapsed_time_bounds:.4f} minutes", flush=True)

        args_list = [
            (plot_id, coords, memmap_path, shape, dtype, crop_outpath, boundaries, True)
            for plot_id, coords in plots.items()
        ]

        # Step 4c: Run multiprocessing
        core_count = min(args.cores_crop, len(args_list))
        print("starting the multiprocessing pool", flush=True)
        print(f"[DEBUG] args_list length: {len(args_list)}", flush=True)
        log_memory_usage("Before starting multiprocessing")

        try:
            with get_context("spawn").Pool(processes=core_count, maxtasksperchild=1) as pool:
                pool.map(crop_worker, args_list)
        except Exception as e:
            print(f"[ERROR] Multiprocessing failed: {e}", flush=True)

        # Step 4d: Clean up shared memory
        os.remove(memmap_path)
        print("Removing memory-mapped file", flush=True)
        
        end_time = time.perf_counter()
        elapsed_time_crop = (end_time - start_time) / 60
        print(f"Elapsed time for cropping plots: {elapsed_time_crop:.4f} minutes\n", flush=True)
    
    end_time = time.perf_counter()
    elapsed_time = (end_time - start_time0) / 60
    print(" --- Summary ---", flush=True)
    if not args.disablegeo:
        print(f"Elapsed time for geocorrection: {elapsed_time_geo:.4f} minutes", flush=True)
    print(f"Elapsed time for merging: {elapsed_time_merge:.4f} minutes", flush=True)
    if not args.disablepcd:
        print(f"Elapsed time for outputting point clouds: {elapsed_time_pcd:.4f} minutes", flush=True)
    if not args.disablecrop:
        print(f"Elapsed time for loading plots: {elapsed_time_plots:.4f} minutes", flush=True)
        print(f"Elapsed time for cropping plots: {elapsed_time_crop:.4f} minutes", flush=True)
    print(f"Elapsed time for full process: {elapsed_time:.4f} minutes", flush=True)
    mem_peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    print(f"[MEMORY] Peak usage: {mem_peak / 1024 / 1024:.2f} GB", flush=True)

if __name__ == "__main__":
    main()

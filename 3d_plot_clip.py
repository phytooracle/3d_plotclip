import argparse
from utils import *
from multiprocessing import Pool, get_context
import time
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

    parser.add_argument('--dynamiccores', 
                        help='Enable dynamic core allocation based on available resources.', 
                        action='store_true')
                        
    return parser.parse_args()

def main():
    args = get_args()
    if not os.path.isdir(args.output):
        os.makedirs(args.output)
    
    partial_out_dir = os.path.join(args.output, "partial_out")
    final_out_dir = os.path.join(args.output, "final_out")
    os.makedirs(partial_out_dir, exist_ok=True)
    os.makedirs(final_out_dir, exist_ok=True)

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
        if args.dynamiccores:
            geo_worker_mem_gb = 5
            core_count = min(estimate_worker_count(geo_worker_mem_gb), len(folder_names))
            print(f"[RESOURCE] Dynamic geo workers: {core_count}", flush=True)
        else:
            core_count = min(args.cores_geo, len(folder_names))
        with Pool(processes=core_count) as pool:
            pool.map(process_folder, args_list)

        end_time = time.perf_counter()
        elapsed_time_geo = (end_time - start_time0) / 60
        print(f"Elapsed time for geocorrection: {elapsed_time_geo:.4f} minutes", flush=True)


    if not args.disablepcd:
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
            del merged_pcd
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

        # Step 4: Collect geocorrected passes
        print("Collecting geocorrected passes...", flush=True)
        geocorrected_dir = os.path.join(args.input, "merged_geocorrected")
        pass_files = []
        for folder in os.listdir(geocorrected_dir):
            folder_path = os.path.join(geocorrected_dir, folder)
            if os.path.isdir(folder_path):
                for fname in os.listdir(folder_path):
                    if fname.endswith('.ply'):
                        pass_files.append(os.path.join(folder_path, fname))

        print(f"[INFO] Found {len(pass_files)} geocorrected passes", flush=True)

        # Prepare args for workers
        worker_args = [(pf, plots, partial_out_dir) for pf in pass_files]
        
        if args.dynamiccores:
            mem_per_worker_gb = 5  # Estimate per pass
            core_count = min(estimate_worker_count(mem_per_worker_gb), len(worker_args))
            print(f"[RESOURCE] Dynamic crop workers: {core_count}", flush=True)
        else:
            core_count = min(args.cores_crop, len(worker_args))

        # Parallel crop
        start_time = time.perf_counter()
        print("Cropping plots from merged point cloud...", flush=True)
        with get_context("spawn").Pool(processes=core_count) as pool:
            pool.map(crop_plots_from_pass, worker_args)
        end_time = time.perf_counter()
        
        elapsed_time_crop = (end_time - start_time) / 60
        print(f"[INFO] Cropping completed in {elapsed_time_crop:.2f} minutes", flush=True)

        # Merge partial plots
        start_time = time.perf_counter()
        merge_partial_plots(partial_out_dir, final_out_dir)
        end_time = time.perf_counter()
        elapsed_time_final_merge = (end_time - start_time) / 60
        print(f"[INFO] Final merging completed in {elapsed_time_final_merge:.2f} minutes", flush=True)
    
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
        print(f"Elapsed time for final merge: {elapsed_time_final_merge:.4f} minutes", flush=True)
    print(f"Elapsed time for full process: {elapsed_time:.4f} minutes", flush=True)
    mem_peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    print(f"[MEMORY] Peak usage: {mem_peak / 1024 / 1024:.2f} GB", flush=True)

if __name__ == "__main__":
    main()

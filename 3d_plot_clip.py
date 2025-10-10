import argparse
from utils import *
import tempfile
import shutil
from multiprocessing import Pool
import time

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)
print("libraries imported", flush=True)

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
                        
    parser.add_argument('-c',
                        '--cores',
                        help='Maximum number of cpus to use in multiprocessing.',
                        type=int,
                        default=20,
                        required=True)

    return parser.parse_args()

def process_folder(args_tuple):
    folder_name, input_path, output_path, transformation, date = args_tuple
    postprocess_single_pass(
        path=input_path,
        outpath=output_path,
        folder=folder_name,
        transformation=transformation,
        current_date=date
    )

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
        core_count = min(args.cores, len(folder_names))
        with Pool(processes=core_count) as pool:
            pool.map(process_folder, args_list)
    end_time = time.perf_counter()
    elapsed_time_geo = end_time - start_time0
    print(f"Elapsed time for geocorrection: {elapsed_time_geo:.4f} seconds", flush=True)

    # Step 2: Merge all geocorrected point clouds
    print("Merging started\n", flush=True)
    start_time = time.perf_counter()
    geocorrected_dir = os.path.join(args.input, "merged_geocorrected")
    merged_pcd = o3d.geometry.PointCloud()
    for pass_id in os.listdir(geocorrected_dir):
        pass_dir = os.path.join(geocorrected_dir, pass_id)
        if os.path.isdir(pass_dir):
            for filename in os.listdir(pass_dir):
                if filename.endswith(".ply"):
                    pcd_path = os.path.join(pass_dir, filename)
                    pcd = o3d.io.read_point_cloud(pcd_path)
                    if not pcd.is_empty():
                        merged_pcd += pcd
                        del pcd
    print("Merging complete\n", flush=True)
    end_time = time.perf_counter()
    elapsed_time_merge = end_time - start_time
    print(f"Elapsed time for merging: {elapsed_time_merge:.4f} seconds", flush=True)
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
            elapsed_time_pcd = end_time - start_time
            print(f"Elapsed time for outputting point clouds: {elapsed_time_pcd:.4f} seconds", flush=True)
        
    
    if not args.disablecrop:
        # Step 3: Load plot definitions
        print("\nLoading plot definitions...", flush=True)
        start_time = time.perf_counter()
        plots = load_plots(args.geojson)
        print("Plots loaded\n", flush=True)
        end_time = time.perf_counter()
        elapsed_time_plots = end_time - start_time
        print(f"Elapsed time for loading plots: {elapsed_time_plots:.4f} seconds", flush=True)

        # Step 4: Crop plots from the transformed cloud
        print("Cropping plots from merged point cloud...", flush=True)
        start_time = time.perf_counter()
        crop_outpath = os.path.join(args.output,"plotclip_out")
        if not os.path.isdir(crop_outpath):
            os.makedirs(crop_outpath)
        crop_and_save_plots(merged_pcd, plots, crop_outpath, args.transformation, force_crop=True)
        end_time = time.perf_counter()
        elapsed_time_crop = end_time - start_time
        print(f"Elapsed time for cropping plots: {elapsed_time_crop:.4f} seconds", flush=True)
    
    end_time = time.perf_counter()
    elapsed_time = end_time - start_time0
    print(f"Elapsed time for geocorrection: {elapsed_time_geo:.4f} seconds", flush=True)
    print(f"Elapsed time for merging: {elapsed_time_merge:.4f} seconds", flush=True)
    print(f"Elapsed time for outputting point clouds: {elapsed_time_pcd:.4f} seconds", flush=True)
    print(f"Elapsed time for loading plots: {elapsed_time_plots:.4f} seconds", flush=True)
    print(f"Elapsed time for cropping plots: {elapsed_time_crop:.4f} seconds", flush=True)
    print(f"Elapsed time for full process: {elapsed_time:.4f} seconds", flush=True)

if __name__ == "__main__":
    main()

import argparse
from utils import *
import tempfile
import shutil
from multiprocessing import Pool

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)
print("libraries imported")

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

def process_folder(folder_name):
    postprocess_single_pass(
        path=args.input,
        outpath=args.input,
        folder=folder_name,
        transformation=args.transformation,
        current_date=args.date
    )

def main():
    args = get_args()
    if not os.path.isdir(args.output):
        os.makedirs(args.output)

    # Step 1: Geocorrect all passes
    if not args.disablegeo:
        merged_dir = os.path.join(args.input, "merged")
        folder_names = [
            f for f in os.listdir(merged_dir)
            if os.path.isdir(os.path.join(merged_dir, f))
        ]
        core_count = min(args.cores, len(folder_names))
        with Pool(processes=core_count) as pool:
            pool.map(process_folder, folder_names)


    # Step 2: Merge all geocorrected point clouds
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
    if not args.disablepcd:
        if args.points:
            merged_pcd_output = merged_pcd
            max_points = args.points * 1_000_000
            current_points = len(merged_pcd_output.points)
            if current_points > max_points:
                print(f"Downsampling merged point cloud from {current_points} to {max_points} points...")
                merged_pcd_output = merged_pcd_output.random_down_sample(max_points / current_points)
                merged_pcd_output_outpath = os.path.join(args.input, args.date + "_" + str(args.points) + "milMax_merged_geocorrected.ply")
                save_pcd(merged_pcd_output, merged_pcd_output_outpath)
                del merged_pcd_output
        # Commented out saving of full-scale point clouds due to extremely large size
        #merged_pcd_outpath = os.path.join(args.input, args.date + "_full" + "_merged_geocorrected.ply")
        #save_pcd(merged_pcd, merged_pcd_outpath)
    
    if not args.disablecrop:
        # Step 3: Load plot definitions
        print("\nLoading plot definitions...")
        plots = load_plots(args.geojson)
        print("Plots loaded\n")

        # Step 4: Crop plots from the transformed cloud
        print("Cropping plots from merged point cloud...")
        crop_and_save_plots(merged_pcd, plots, args.output, args.transformation, force_crop=True)

if __name__ == "__main__":
    main()

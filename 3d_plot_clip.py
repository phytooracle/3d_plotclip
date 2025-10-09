import argparse
from utils import *
import tempfile
import shutil

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

    return parser.parse_args()


def main():
    args = get_args()

    if not os.path.isdir(args.output):
        os.makedirs(args.output)

    # Step 1: Geocorrect all passes
    for folder_name in os.listdir(os.path.join(args.input, "merged")):
        if os.path.isdir(os.path.join(args.input, "merged", folder_name)):
            print(f"Geocorrecting pass: {folder_name}")
            postprocess_single_pass(
                path=args.input,
                outpath=args.input,
                folder=folder_name,
                transformation=args.transformation,
                current_date=args.date
            )

    # Step 2: Load plot definitions
    print("\nLoading plot definitions...")
    plots = load_plots(args.geojson)
    print("Plots loaded\n")

    # Step 3: Merge all geocorrected point clouds
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
    merged_pcd_downsampled = merged_pcd.voxel_down_sample(voxel_size=5)
    merged_pcd_downsampled_outpath = os.path.join(args.input,args.date + "_full_merged_geocorrected_downsampled.ply")
    o3d.io.write_point_cloud(merged_pcd_downsampled_outpath, merged_pcd_downsampled)

    # Step 4: Crop plots from the merged cloud
    print("Cropping plots from merged point cloud...")
    crop_and_save_plots(merged_pcd, plots, args.output, "merged", force_crop=True)


if __name__ == "__main__":
    main()
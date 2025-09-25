#!/usr/bin/env python3

import h5py
import numpy as np
import cv2
import argparse
import os
import sys
from pathlib import Path

def extract_video_from_hdf5(hdf5_path, output_dir=None, camera_names=None, fps=30):
    """
    Extract videos from ACT HDF5 dataset
    
    Args:
        hdf5_path: Path to HDF5 file
        output_dir: Output directory (default: same as HDF5 file)
        camera_names: List of camera names to extract (default: all found)
        fps: Video framerate (default: 30)
    """
    
    if output_dir is None:
        output_dir = Path(hdf5_path).parent
    else:
        output_dir = Path(output_dir)
    
    output_dir.mkdir(exist_ok=True)
    
    episode_name = Path(hdf5_path).stem
    print(f"🎬 Extracting videos from: {episode_name}")
    
    with h5py.File(hdf5_path, 'r') as f:
        # Find all image datasets
        image_datasets = {}
        
        def find_images(name, obj):
            if isinstance(obj, h5py.Dataset) and 'image' in name.lower():
                # Check if it's video data (has frames dimension)
                if len(obj.shape) >= 3 and obj.shape[-1] == 3:
                    camera_name = name.split('/')[-1]  # Get last part of path
                    image_datasets[camera_name] = name
                    print(f"  📹 Found camera '{camera_name}': {obj.shape}")
        
        f.visititems(find_images)
        
        if not image_datasets:
            print("❌ No video datasets found!")
            return
            
        # Filter by requested cameras
        if camera_names:
            image_datasets = {k: v for k, v in image_datasets.items() if k in camera_names}
            
        # Extract videos
        for camera_name, dataset_path in image_datasets.items():
            dataset = f[dataset_path]
            num_frames, height, width, channels = dataset.shape
            
            # Create video writer
            video_path = output_dir / f"{episode_name}_{camera_name}.mp4"
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(str(video_path), fourcc, fps, (width, height))
            
            print(f"  🎥 Extracting {camera_name}: {num_frames} frames → {video_path}")
            
            # Write frames
            for i in range(num_frames):
                frame = dataset[i]  # Shape: (H, W, 3)
                
                # Convert RGB to BGR for OpenCV
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                video_writer.write(frame_bgr)
                
                if i % 50 == 0:
                    print(f"    Progress: {i}/{num_frames} ({100*i/num_frames:.1f}%)")
            
            video_writer.release()
            print(f"  ✅ Saved: {video_path}")
    
    print(f"🎉 Video extraction completed! Output: {output_dir}")

def batch_extract_videos(data_dir, output_dir=None, camera_names=None, max_files=None):
    """Extract videos from all HDF5 files in directory"""
    
    data_path = Path(data_dir)
    hdf5_files = list(data_path.glob("*.hdf5"))
    
    if not hdf5_files:
        print(f"❌ No HDF5 files found in {data_dir}")
        return
        
    if max_files:
        hdf5_files = hdf5_files[:max_files]
        
    print(f"🚀 Batch extracting {len(hdf5_files)} files...")
    
    for i, hdf5_file in enumerate(hdf5_files, 1):
        print(f"\n[{i}/{len(hdf5_files)}] Processing {hdf5_file.name}")
        try:
            extract_video_from_hdf5(hdf5_file, output_dir, camera_names)
        except Exception as e:
            print(f"❌ Failed to process {hdf5_file.name}: {e}")

def main():
    parser = argparse.ArgumentParser(description="Extract videos from ACT HDF5 datasets")
    parser.add_argument("input", nargs='?', default="/home/jason/ws/catkin_ws/src/act_data", 
                        help="HDF5 file or directory containing HDF5 files (default: /home/jason/ws/catkin_ws/src/act_data)")
    parser.add_argument("-o", "--output", help="Output directory (default: input_dir/videos)")
    parser.add_argument("-c", "--cameras", nargs="+", help="Camera names to extract (e.g., top side)")
    parser.add_argument("--fps", type=int, default=30, help="Video FPS (default: 30)")
    parser.add_argument("--max-files", type=int, help="Max number of files to process")
    parser.add_argument("--sample", action="store_true", help="Extract only first 3 files as samples")
    
    args = parser.parse_args()
    
    input_path = Path(args.input)
    
    if input_path.is_file():
        # Single file
        extract_video_from_hdf5(input_path, args.output, args.cameras, args.fps)
    elif input_path.is_dir():
        # Directory - set default output to input_dir/videos if not specified
        if args.output is None:
            args.output = input_path / "videos"
            
        # Process all files by default (no max_files limit unless specified)
        max_files = 3 if args.sample else args.max_files
        
        print(f"🚀 Processing all HDF5 files in: {input_path}")
        print(f"📁 Output directory: {args.output}")
        print(f"🎬 Press Ctrl+C to interrupt processing at any time")
        
        try:
            batch_extract_videos(input_path, args.output, args.cameras, max_files)
        except KeyboardInterrupt:
            print("\n⚠️  Processing interrupted by user")
            print("✅ Videos extracted so far have been saved")
    else:
        print(f"❌ Input path not found: {input_path}")
        sys.exit(1)

if __name__ == "__main__":
    main()

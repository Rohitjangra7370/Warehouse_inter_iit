#!/usr/bin/env python3
"""
Map Wall Generator for Warehouse Robot
Creates virtual walls at specified distance from all obstacles (racks, walls)
to enforce tape boundary constraints.

Usage:
    python3 map_wall_generator.py input_map.pgm --distance 0.40 --output output_map.pgm
"""

import cv2
import numpy as np
import yaml
import argparse
import sys
from pathlib import Path

class MapWallGenerator:
    def __init__(self, input_map_path, wall_distance_meters=0.40, resolution=None):
        """
        Initialize map wall generator
        
        Args:
            input_map_path: Path to input .pgm map file
            wall_distance_meters: Distance from obstacles to place virtual walls (meters)
            resolution: Map resolution (m/pixel), auto-detected from YAML if None
        """
        self.input_path = Path(input_map_path)
        self.wall_distance = wall_distance_meters
        
        # Load map image
        self.map_img = cv2.imread(str(self.input_path), cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            raise ValueError(f"Failed to load map: {self.input_path}")
        
        # Load or set resolution
        self.resolution = resolution
        if self.resolution is None:
            self.resolution = self._load_resolution()
        
        print(f"📄 Loaded map: {self.input_path}")
        print(f"   Size: {self.map_img.shape[1]} x {self.map_img.shape[0]} pixels")
        print(f"   Resolution: {self.resolution} m/pixel")
        
        # Calculate wall distance in pixels
        self.wall_pixels = int(self.wall_distance / self.resolution)
        print(f"   Wall distance: {self.wall_distance}m = {self.wall_pixels} pixels")
    
    def _load_resolution(self):
        """Load resolution from YAML file"""
        yaml_path = self.input_path.with_suffix('.yaml')
        
        if not yaml_path.exists():
            print(f"⚠️  YAML file not found: {yaml_path}")
            print(f"   Using default resolution: 0.05 m/pixel")
            return 0.05
        
        try:
            with open(yaml_path, 'r') as f:
                map_meta = yaml.safe_load(f)
            
            resolution = map_meta.get('resolution', 0.05)
            print(f"✅ Loaded resolution from YAML: {resolution} m/pixel")
            return resolution
        
        except Exception as e:
            print(f"⚠️  Error reading YAML: {e}")
            print(f"   Using default resolution: 0.05 m/pixel")
            return 0.05
    
    def generate_walls(self, method='dilation'):
        """
        Generate virtual walls around obstacles
        
        Args:
            method: 'dilation' (faster) or 'distance_transform' (more precise)
        
        Returns:
            Processed map image with virtual walls
        """
        print(f"\n🔧 Generating virtual walls...")
        print(f"   Method: {method}")
        
        # Threshold map: free space (white), obstacles (black), unknown (gray)
        # Typical ROS maps: 0-100 = occupied, 101-254 = free, 255 = unknown
        
        # Create binary map: 0 = obstacle/unknown, 255 = free
        _, free_space = cv2.threshold(self.map_img, 200, 255, cv2.THRESH_BINARY)
        
        # Invert: obstacles = 255, free = 0
        obstacles = cv2.bitwise_not(free_space)
        
        if method == 'dilation':
            result = self._dilate_obstacles(obstacles)
        elif method == 'distance_transform':
            result = self._distance_transform_walls(obstacles)
        else:
            raise ValueError(f"Unknown method: {method}")
        
        print(f"✅ Virtual walls generated!")
        return result
    
    def _dilate_obstacles(self, obstacles):
        """
        Simple dilation method: expand all obstacles by wall_distance
        Fast but creates circular walls around obstacles
        """
        # Create circular kernel
        kernel_size = self.wall_pixels * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                          (kernel_size, kernel_size))
        
        # Dilate obstacles
        dilated = cv2.dilate(obstacles, kernel, iterations=1)
        
        # Invert back: free space = white, walls = black
        result = cv2.bitwise_not(dilated)
        
        return result
    
    def _distance_transform_walls(self, obstacles):
        """
        Distance transform method: create walls at exact distance from obstacles
        More precise but slower
        """
        # Compute distance transform (distance to nearest obstacle)
        dist_transform = cv2.distanceTransform(cv2.bitwise_not(obstacles), 
                                              cv2.DIST_L2, 5)
        
        # Create walls at specified distance
        # Pixels within wall_distance become obstacles
        _, walls = cv2.threshold(dist_transform, self.wall_pixels, 255, 
                                cv2.THRESH_BINARY_INV)
        
        walls = walls.astype(np.uint8)
        
        # Combine with original obstacles
        combined = cv2.bitwise_or(obstacles, walls)
        
        # Invert: free space = white, walls = black
        result = cv2.bitwise_not(combined)
        
        return result
    
    def add_safety_margin(self, map_img, margin_pixels=10):
        """
        Add extra safety margin around walls (optional)
        
        Args:
            map_img: Input map
            margin_pixels: Additional pixels to add around obstacles
        
        Returns:
            Map with safety margin
        """
        if margin_pixels <= 0:
            return map_img
        
        print(f"\n🛡️  Adding safety margin: {margin_pixels} pixels")
        
        # Invert
        obstacles = cv2.bitwise_not(map_img)
        
        # Small dilation for safety margin
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                          (margin_pixels*2+1, margin_pixels*2+1))
        dilated = cv2.dilate(obstacles, kernel, iterations=1)
        
        # Invert back
        result = cv2.bitwise_not(dilated)
        
        return result
    
    def visualize_comparison(self, original, processed):
        """Create side-by-side comparison visualization"""
        # Resize if needed for display
        max_height = 800
        if original.shape[0] > max_height:
            scale = max_height / original.shape[0]
            original = cv2.resize(original, None, fx=scale, fy=scale)
            processed = cv2.resize(processed, None, fx=scale, fy=scale)
        
        # Create side-by-side comparison
        comparison = np.hstack([original, processed])
        
        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(comparison, "ORIGINAL", (10, 30), font, 1, 0, 2)
        cv2.putText(comparison, "WITH WALLS", (original.shape[1] + 10, 30), 
                   font, 1, 0, 2)
        
        return comparison
    
    def save_map(self, map_img, output_path, copy_yaml=True):
        """
        Save processed map and optionally copy/update YAML
        
        Args:
            map_img: Processed map image
            output_path: Output .pgm file path
            copy_yaml: Whether to copy/update YAML metadata
        """
        output_path = Path(output_path)
        
        # Save PGM
        success = cv2.imwrite(str(output_path), map_img)
        
        if not success:
            raise IOError(f"Failed to save map: {output_path}")
        
        print(f"\n💾 Saved processed map: {output_path}")
        
        # Copy/update YAML
        if copy_yaml:
            input_yaml = self.input_path.with_suffix('.yaml')
            output_yaml = output_path.with_suffix('.yaml')
            
            if input_yaml.exists():
                with open(input_yaml, 'r') as f:
                    metadata = yaml.safe_load(f)
                
                # Update image filename
                metadata['image'] = output_path.name
                
                with open(output_yaml, 'w') as f:
                    yaml.dump(metadata, f, default_flow_style=False)
                
                print(f"💾 Saved metadata: {output_yaml}")
            else:
                print(f"⚠️  No YAML file to copy from {input_yaml}")
        
        return output_path


def main():
    parser = argparse.ArgumentParser(
        description='Generate virtual walls in occupancy grid map',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage (400mm walls)
  python3 map_wall_generator.py arena_map.pgm
  
  # Custom wall distance
  python3 map_wall_generator.py arena_map.pgm --distance 0.50
  
  # Specify output filename
  python3 map_wall_generator.py arena_map.pgm --output safe_map.pgm
  
  # Add safety margin
  python3 map_wall_generator.py arena_map.pgm --margin 10
  
  # Use distance transform (more precise)
  python3 map_wall_generator.py arena_map.pgm --method distance_transform
  
  # Show visualization
  python3 map_wall_generator.py arena_map.pgm --visualize
        """
    )
    
    parser.add_argument('input_map', 
                       help='Input .pgm map file')
    parser.add_argument('--distance', '-d', type=float, default=0.40,
                       help='Wall distance in meters (default: 0.40)')
    parser.add_argument('--output', '-o', 
                       help='Output .pgm file (default: input_walls.pgm)')
    parser.add_argument('--method', '-m', 
                       choices=['dilation', 'distance_transform'],
                       default='dilation',
                       help='Wall generation method (default: dilation)')
    parser.add_argument('--margin', type=int, default=0,
                       help='Additional safety margin in pixels (default: 0)')
    parser.add_argument('--visualize', '-v', action='store_true',
                       help='Show before/after visualization')
    parser.add_argument('--resolution', type=float,
                       help='Map resolution in m/pixel (auto-detect if not specified)')
    
    args = parser.parse_args()
    
    # Determine output path
    if args.output:
        output_path = args.output
    else:
        input_path = Path(args.input_map)
        output_path = input_path.parent / f"{input_path.stem}_walls{input_path.suffix}"
    
    try:
        # Initialize generator
        generator = MapWallGenerator(
            args.input_map, 
            wall_distance_meters=args.distance,
            resolution=args.resolution
        )
        
        # Generate walls
        processed_map = generator.generate_walls(method=args.method)
        
        # Add safety margin if specified
        if args.margin > 0:
            processed_map = generator.add_safety_margin(processed_map, args.margin)
        
        # Save result
        generator.save_map(processed_map, output_path)
        
        # Visualize if requested
        if args.visualize:
            print("\n👁️  Displaying comparison...")
            comparison = generator.visualize_comparison(generator.map_img, processed_map)
            cv2.imshow('Map Comparison: Original (left) vs With Walls (right)', comparison)
            print("   Press any key to close...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        print("\n✅ Done!")
        print(f"\n📝 Next steps:")
        print(f"   1. Verify the map looks correct: eog {output_path}")
        print(f"   2. Use in navigation: ros2 launch <pkg> navigation.launch.py map:={output_path.replace('.pgm', '.yaml')}")
        
    except Exception as e:
        print(f"\n❌ Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# map_inflator.py

import cv2
import numpy as np
import yaml
import sys

def inflate_map(input_pgm, output_pgm, inflation_meters=0.40):
    """
    Inflate obstacles in PGM map by specified distance
    """
    # Read map
    img = cv2.imread(input_pgm, cv2.IMREAD_GRAYSCALE)
    
    # Read map metadata
    yaml_file = input_pgm.replace('.pgm', '.yaml')
    with open(yaml_file, 'r') as f:
        map_meta = yaml.safe_load(f)
    
    resolution = map_meta['resolution']  # meters per pixel
    
    # Convert inflation distance to pixels
    inflation_pixels = int(inflation_meters / resolution)
    
    print(f"Map resolution: {resolution} m/pixel")
    print(f"Inflating obstacles by {inflation_meters}m = {inflation_pixels} pixels")
    
    # Threshold: occupied cells (< 250)
    _, binary = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)
    
    # Invert: obstacles = white
    obstacles = cv2.bitwise_not(binary)
    
    # Dilate (inflate obstacles)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                       (inflation_pixels*2, inflation_pixels*2))
    inflated = cv2.dilate(obstacles, kernel, iterations=1)
    
    # Invert back
    result = cv2.bitwise_not(inflated)
    
    # Save
    cv2.imwrite(output_pgm, result)
    
    # Copy YAML with new filename
    output_yaml = output_pgm.replace('.pgm', '.yaml')
    map_meta['image'] = output_pgm.split('/')[-1]
    with open(output_yaml, 'w') as f:
        yaml.dump(map_meta, f)
    
    print(f"✅ Inflated map saved: {output_pgm}")
    print(f"✅ Metadata saved: {output_yaml}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 map_inflator.py <input.pgm> [inflation_meters]")
        sys.exit(1)
    
    input_pgm = sys.argv[1]
    inflation = float(sys.argv[2]) if len(sys.argv) > 2 else 0.40
    
    output_pgm = input_pgm.replace('.pgm', '_inflated.pgm')
    inflate_map(input_pgm, output_pgm, inflation)

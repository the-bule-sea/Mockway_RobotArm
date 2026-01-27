#!/usr/bin/env python3
"""
Generate ArUco marker texture for Gazebo
"""

import cv2
import numpy as np
import os
import sys


def generate_aruco_marker(marker_id=0, marker_size=200, border_bits=1):
    """
    Generate an ArUco marker image

    Args:
        marker_id: ID of the marker to generate
        marker_size: Size of the output image in pixels
        border_bits: Number of border bits (default 1 for 4x4 markers)
    """
    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # Generate marker image
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, borderBits=border_bits)

    return marker_img


def main():
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)

    # Output directory
    output_dir = os.path.join(package_dir, 'models', 'aruco_marker_4x4', 'materials', 'textures')

    # Create directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Generate marker ID 0 from DICT_4X4_50
    marker_id = 0
    marker_size = 400  # High resolution for better detection

    print(f'Generating ArUco marker ID {marker_id} from DICT_4X4_50...')
    marker_img = generate_aruco_marker(marker_id, marker_size)

    # Save marker
    output_path = os.path.join(output_dir, f'aruco_marker_{marker_id}.png')
    cv2.imwrite(output_path, marker_img)
    print(f'Marker saved to: {output_path}')

    # Generate a few more markers for testing
    for i in range(1, 5):
        marker_img = generate_aruco_marker(i, marker_size)
        output_path = os.path.join(output_dir, f'aruco_marker_{i}.png')
        cv2.imwrite(output_path, marker_img)
        print(f'Generated marker ID {i}')

    print('\nArUco markers generated successfully!')
    print(f'Total markers: 5 (IDs 0-4)')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Simple test script for ArUco detection without ROS
Useful for testing camera and marker detection
"""

import cv2
import numpy as np
import argparse


def main():
    parser = argparse.ArgumentParser(description='Test ArUco marker detection')
    parser.add_argument('--camera', type=int, default=0, help='Camera device ID')
    parser.add_argument('--marker_size', type=float, default=0.03, help='Marker size in meters')
    parser.add_argument('--dict', type=str, default='4X4_50', help='ArUco dictionary')
    args = parser.parse_args()

    # Initialize ArUco detector
    aruco_dict_map = {
        '4X4_50': cv2.aruco.DICT_4X4_50,
        '4X4_100': cv2.aruco.DICT_4X4_100,
        '4X4_250': cv2.aruco.DICT_4X4_250,
        '4X4_1000': cv2.aruco.DICT_4X4_1000,
    }

    if args.dict not in aruco_dict_map:
        print(f'Unknown dictionary: {args.dict}')
        print(f'Available: {list(aruco_dict_map.keys())}')
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[args.dict])
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # Open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f'Cannot open camera {args.camera}')
        return

    print(f'ArUco Detection Test')
    print(f'Dictionary: {args.dict}')
    print(f'Marker size: {args.marker_size}m')
    print(f'Camera: {args.camera}')
    print(f'\nPress "q" to quit\n')

    # Camera matrix (rough estimate, should be calibrated for accurate pose)
    # For a typical webcam with ~60 degree FOV
    ret, frame = cap.read()
    if not ret:
        print('Cannot read from camera')
        return

    h, w = frame.shape[:2]
    focal_length = w / (2 * np.tan(np.radians(30)))  # Assuming 60 deg FOV
    camera_matrix = np.array([
        [focal_length, 0, w/2],
        [0, focal_length, h/2],
        [0, 0, 1]
    ], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1))

    print(f'Camera resolution: {w}x{h}')
    print('Note: Using estimated camera calibration. For accurate pose, calibrate your camera.\n')

    while True:
        ret, frame = cap.read()
        if not ret:
            print('Cannot read frame')
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = detector.detectMarkers(gray)

        # Draw detected markers
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, args.marker_size, camera_matrix, dist_coeffs)

            for i, marker_id in enumerate(ids):
                # Draw axis
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                rvecs[i], tvecs[i], args.marker_size * 0.5)

                # Display info
                tvec = tvecs[i][0]
                info = f'ID:{marker_id[0]} Pos:({tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f})m'
                cv2.putText(frame, info, (10, 30 + i*30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Calculate distance
                distance = np.linalg.norm(tvec)
                dist_text = f'Distance: {distance:.3f}m'
                cv2.putText(frame, dist_text, (10, 60 + i*30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(frame, 'No markers detected', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Display FPS
        cv2.putText(frame, f'Press "q" to quit', (10, h-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow('ArUco Detection Test', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print('\nTest completed')


if __name__ == '__main__':
    main()

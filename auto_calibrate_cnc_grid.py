import numpy as np
import cv2
import serial
import time
import argparse

# =============================================================================
# Automatic CNC Calibration with ArUco Marker Detection - Full Grid
# This script moves the CNC through a full grid of points (not just borders)
# and attempts to detect an ArUco marker at each point within a timeout.
# =============================================================================

def parse_args():
    parser = argparse.ArgumentParser(description='Auto Calibrate CNC full-grid with ArUco marker')
    parser.add_argument('--width', type=int, default=300, help='CNC width (mm)')
    parser.add_argument('--height', type=int, default=565, help='CNC height (mm)')
    parser.add_argument('--spacing', type=int, default=50, help='Grid spacing (mm)')
    parser.add_argument('--aruco-id', type=int, default=2, help='ID of the ArUco marker')
    parser.add_argument('--timeout', type=float, default=3.0, help='Max detection time per point (s)')
    parser.add_argument('--attempts', type=int, default=200, help='Max detection attempts per point')
    parser.add_argument('--port', type=str, default='/dev/grbl', help='Serial port for CNC')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for CNC serial')
    return parser.parse_args()


def detector(cap, aruco_detector, x_cnc, y_cnc, aruco_id, timeout, max_attempts, calib_data):
    """
    Attempt to detect the ArUco marker at the current CNC position.
    If detection fails within timeout or attempts, skip this point.
    """
    print(f"[INFO] Detecting marker at CNC ({x_cnc}, {y_cnc})...")
    start_time = time.time()
    attempts = 0
    detected = False

    while not detected:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to grab frame. Retrying...")
            continue

        corners, ids, _ = aruco_detector.detectMarkers(frame)
        frame_display = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        cv2.putText(frame_display, f"CNC({x_cnc},{y_cnc})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Calibration", frame_display)
        cv2.waitKey(1)

        if ids is not None and aruco_id in ids:
            idx = np.where(ids == aruco_id)[0][0]
            x_cam, y_cam = corners[idx][0][0]
            calib_data['cnc'].append([x_cnc, y_cnc])
            calib_data['cam'].append([float(x_cam), float(y_cam)])
            print(f"[SUCCESS] Captured CNC({x_cnc},{y_cnc}) -> CAM({int(x_cam)},{int(y_cam)})")
            detected = True
        else:
            attempts += 1
            elapsed = time.time() - start_time
            if elapsed > timeout or attempts >= max_attempts:
                print(f"[TIMEOUT] No marker detected at CNC({x_cnc},{y_cnc}) after {elapsed:.1f}s. Skipping.")
                break

    return detected


def main():
    args = parse_args()

    # Initialize video capture and serial
    cap = cv2.VideoCapture(2)
    cnc = serial.Serial(args.port, args.baud, timeout=1)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict)
    calib_data = {'cnc': [], 'cam': []}

    # Create a named window once
    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)

    try:
        # Full-grid traversal
        y_points = range(0, args.height + 1, args.spacing)
        x_points = range(0, args.width + 1, args.spacing)

        for y in y_points:
            for x in x_points:
                cnc.write(f"G0 X{x} Y{y}\n".encode())
                time.sleep(2.5)  # give CNC time to move
                detector(cap, aruco_detector, x, y, args.aruco_id,
                         args.timeout, args.attempts, calib_data)

    except KeyboardInterrupt:
        print("[INFO] Calibration interrupted by user.")
    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        np.save("calib_data.npy", calib_data)
        print(f"[INFO] Calibration data saved to calib_data.npy ({len(calib_data['cnc'])} points)!")
        cnc.close()

if __name__ == '__main__':
    main()

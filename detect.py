import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture(0)

camera_matrix = np.array([
    [1000, 0, 320],
    [0, 1000, 240],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((5, 1), dtype=np.float32)
marker_length = 0.05

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)  # Novo objeto detector

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )

        for i in range(len(ids)):
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length)
            
            x, y, z = tvecs[i][0]
            cv2.putText(frame, f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"ID: {ids[i][0]}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow('ArUco Detection', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
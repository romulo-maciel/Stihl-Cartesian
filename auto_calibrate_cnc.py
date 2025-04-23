# calibrate_cnc.py
import numpy as np
import cv2
import serial
import time

CNC_WIDTH = 300  # Medida da barra onde está preso o end effector em mm
CNC_HEIGHT = 565  # Comprimento do frame da CNC em mm

ARUCO_ID = 8  # ID do marcador no end effector
calib_data = {"cnc": [], "cam": []}
cap = cv2.VideoCapture(0)
cnc = serial.Serial('/dev/grbl', 115200)
aruco_detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))


def detector(x_cnc = 0, y_cnc = 0):
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erro na captura do frame")
            continue
            
        corners, ids, _ = aruco_detector.detectMarkers(frame)
        frame_display = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        # cv2.putText(frame_display, f"Ponto: ({0}, {y_cnc})", (10, 30), 
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if ids is not None and ARUCO_ID in ids:
            idx = np.where(ids == ARUCO_ID)[0][0]
            x_cam = corners[idx][0][0][0]
            y_cam = corners[idx][0][0][1]
            cv2.putText(frame_display, f"Marker: ({int(x_cam)}, {int(y_cam)})", 
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow("Calibration", frame_display)
        # key = cv2.waitKey(1) & 0xFF

        # Capturar posição
        if ids is not None and ARUCO_ID in ids:
            idx = np.where(ids == ARUCO_ID)[0][0]
            x_cam = corners[idx][0][0][0]
            y_cam = corners[idx][0][0][1]
            calib_data["cnc"].append([x_cnc, y_cnc])
            calib_data["cam"].append([x_cam, y_cam])
            print(f"Capturado: CNC ({x_cnc}, {y_cnc}) -> Cam ({int(x_cam)}, {int(y_cam)})")
            break
        else:
            print("Marcador ArUco não detectado. Tente novamente.")
            cnc.write(f"G0 X{(x_cnc + 1) if x_cnc > 0 else 0} Y{(y_cnc + 1) if y_cnc > 0 else 0}\n".encode())
            time.sleep(1)  

for y_cnc in range( 0, CNC_HEIGHT,  50):
    cnc.write(f"G0 X{0} Y{y_cnc}\n".encode())
    print(f"Posicionando em ({0}, {y_cnc}).")
    time.sleep(1.5) 
    detector(0, y_cnc)

for x_cnc in range(0, CNC_WIDTH, 50):
    cnc.write(f"G0 X{x_cnc} Y{CNC_HEIGHT}\n".encode())
    print(f"Posicionando em ({x_cnc}, {CNC_HEIGHT}).")
    time.sleep(1.5) 
    detector(x_cnc, CNC_HEIGHT)

for y_cnc in range(CNC_HEIGHT, 0, -50):
    cnc.write(f"G0 X{CNC_WIDTH} Y{y_cnc}\n".encode())
    print(f"Posicionando em ({CNC_WIDTH}, {y_cnc}).")
    time.sleep(1.5) 
    detector(CNC_WIDTH, y_cnc)

for x_cnc in range(CNC_WIDTH, 0, -50):
    cnc.write(f"G0 X{x_cnc} Y{0}\n".encode())
    print(f"Posicionando em ({x_cnc}, {0}).")
    time.sleep(1.5) 
    detector(x_cnc, 0)

cap.release()
cv2.destroyAllWindows()
np.save("calib_data.npy", calib_data)
print("Dados salvos em calib_data.npy")

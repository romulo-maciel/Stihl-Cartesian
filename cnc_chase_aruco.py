import cv2
import cv2.aruco as aruco
import numpy as np
import serial
import time

# Configurações da CNC (GRBL)
PORT = '/dev/grbl'
BAUDRATE = 115200

# Configurações da Câmera
CAMERA_ID = 0
TARGET_ID = 42  # ID do ArUco a ser perseguido
MAX_SPEED = 1500  # mm/min

# Configurações do ArUco
MARKER_LENGTH = 0.05  # metros
CAMERA_MATRIX = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)

class CNCDriver:
    def __init__(self):
        self.ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        self._send_command("G21")  # Unidades em mm
        self._send_command("G90")  # Modo absoluto

    def _send_command(self, cmd):
        self.ser.write(f"{cmd}\n".encode())
        while True:
            response = self.ser.readline().decode().strip()
            if 'ok' in response or 'error' in response:
                break

    def move_to(self, x, y):
        self._send_command(f"G0 X{x:.2f} Y{y:.2f}")

    def close(self):
        self.ser.close()

def main():
    cap = cv2.VideoCapture(CAMERA_ID)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())
    cnc = CNCDriver()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Detecta e desenha todos os marcadores
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            
            if ids is not None:
                # Desenha contornos e IDs de todos os marcadores
                aruco.drawDetectedMarkers(frame, corners, ids)

                # Busca pelo marcador alvo
                if TARGET_ID in ids:
                    idx = np.where(ids == TARGET_ID)[0][0]
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        [corners[idx]], MARKER_LENGTH, CAMERA_MATRIX, DIST_COEFFS
                    )

                    # Desenha eixos do marcador alvo
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec[0], tvec[0], MARKER_LENGTH)

                    # Exibe coordenadas
                    x_cam, y_cam, z_cam = tvec[0][0]
                    x_cnc = (x_cam * 1000) + 550   # Conversão para mm
                    y_cnc = (y_cam * 1000) + 550

                    # print(f"Marker ID: {TARGET_ID} - X: {x_cam:.2f} Y: {y_cam:.2f} Z: {z_cam:.2f} | CNC: X: {x_cnc:.2f} Y: {y_cnc:.2f}")

                    cv2.putText(frame, f"Aruco: X:{x_cam:.1f} Y:{y_cam:.1f} Z:{z_cam:.1f}", 
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (127, 255, 0), 2)

                    cv2.putText(frame, f"CNC Target: X:{x_cnc:.1f}mm Y:{y_cnc:.1f}mm", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # Move CNC
                    cnc.move_to(x_cnc, y_cnc)

            cv2.imshow('ArUco Tracking', frame)
            if cv2.waitKey(1) == ord('q'):
                break

    finally:
        cnc.close()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
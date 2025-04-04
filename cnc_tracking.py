# cnc_tracking.py
import cv2
import numpy as np
import serial
import joblib

# Configurações
ARUCO_ID = 8  # ID do marcador alvo
SIM_SIZE = 400  # Tamanho da janela de simulação (pixels)
WORKSPACE_SIZE = 1000  # Tamanho do espaço de trabalho em mm (1000x1000mm)

class CNCTracker:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cnc = serial.Serial('/dev/grbl', 115200)
        self.aruco_detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        self.model, self.poly = joblib.load("calib_model.pkl")
        self.current_pos = (0, 0)  # Posição atual da CNC (assume home em (0,0))
        
        # Configuração da simulação
        self.scale = SIM_SIZE / WORKSPACE_SIZE  # 2 pixels/mm
        self.sim_bg = np.ones((SIM_SIZE, SIM_SIZE, 3), dtype=np.uint8) * 255  # Fundo branco

    def draw_simulation(self, target_pos=None):
        # Cria uma cópia do fundo
        frame = self.sim_bg.copy()
        
        # Desenha grade
        for i in range(0, WORKSPACE_SIZE + 1, 50):
            pos = int(i * self.scale)
            cv2.line(frame, (pos, 0), (pos, SIM_SIZE), (200, 200, 200), 1)
            cv2.line(frame, (0, pos), (SIM_SIZE, pos), (200, 200, 200), 1)
        
        # Converte coordenadas CNC para pixels (inverte Y)
        x_px = int(self.current_pos[0] * self.scale)
        y_px = SIM_SIZE - int(self.current_pos[1] * self.scale)
        
        # Desenha end effector
        cv2.circle(frame, (x_px, y_px), 10, (0, 0, 255), -1)
        cv2.putText(frame, f"CNC: ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f})", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Desenha alvo se detectado
        if target_pos is not None:
            tx_px = int(target_pos[0] * self.scale)
            ty_px = SIM_SIZE - int(target_pos[1] * self.scale)
            cv2.circle(frame, (tx_px, ty_px), 8, (0, 255, 0), -1)
            cv2.putText(frame, f"Target: ({target_pos[0]:.1f}, {target_pos[1]:.1f})", 
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        return frame

    def track(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # Detecta marcadores
            corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            target_pos = None
            
            if ids is not None and ARUCO_ID in ids:
                idx = np.where(ids == ARUCO_ID)[0][0]
                x_cam = corners[idx][0][0][0]
                y_cam = corners[idx][0][0][1]
                
                # Aplica correção polinomial
                cam_coords = np.array([[x_cam, y_cam]])
                cam_poly = self.poly.transform(cam_coords)
                x_cnc, y_cnc = self.model.predict(cam_poly)[0]
                target_pos = (x_cnc, y_cnc)
                
                # Move CNC e atualiza posição atual
                self.cnc.write(f"G0 X{x_cnc:.2f} Y{y_cnc:.2f} F500\n".encode())
                self.current_pos = (x_cnc, y_cnc)
                
                # Desenha no frame da câmera
                cv2.aruco.drawDetectedMarkers(frame, [corners[idx]], np.array([ARUCO_ID]))
                cv2.putText(frame, f"Target Detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Atualiza janelas
            sim_frame = self.draw_simulation(target_pos)
            cv2.imshow('Camera Feed', frame)
            cv2.imshow('CNC Simulation', sim_frame)
            
            if cv2.waitKey(1) == ord('q'):
                break

        self.cnc.close()
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    tracker = CNCTracker()
    tracker.track()
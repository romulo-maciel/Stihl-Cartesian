import serial
import time

class CNCController:
    def __init__(self, port='/dev/grbl', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.cnc = None

    def connect(self):
        self.cnc = serial.Serial(self.port, self.baudrate, timeout=1)
        time.sleep(2)  # Aguarda inicialização do GRBL
        self.flush_buffer()

    def flush_buffer(self):
        self.cnc.reset_input_buffer()
        self.cnc.reset_output_buffer()

    def send_command(self, command):
        self.cnc.write(f"{command}\n".encode())
        response = self.cnc.readline().decode().strip()
        while 'ok' not in response and 'error' not in response:
            response += self.cnc.readline().decode().strip()
        return response

    def safe_move(self):
        # Homing (requer limites configurados)
        # print("Realizando homing...")
        # self.send_command("$H")
        # print("Homing concluído.")

        # Move para X10 Y10 (em mm)
        print("Movendo para X10 Y10...")
        self.send_command("G21")      # Unidades em mm
        self.send_command("G90")      # Modo absoluto
        self.send_command("G0 X1000 Y1000 F500")  # Movimento
        print("Movimento concluído.")

    def disconnect(self):
        if self.cnc:
            self.cnc.close()
            print("Conexão encerrada.")

if __name__ == "__main__":
    cnc = CNCController(port='/dev/grbl')
    try:
        cnc.connect()
        cnc.safe_move()
    except Exception as e:
        print(f"Erro: {e}")
    finally:
        cnc.disconnect()

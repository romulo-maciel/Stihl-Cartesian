import serial
import argparse
import time

def main():
    parser = argparse.ArgumentParser(description='Terminal GRBL para CNC')
    parser.add_argument('-p', '--port', default='/dev/grbl', help='Porta serial (padrão: /dev/grbl)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='Baud rate (padrão: 115200)')
    args = parser.parse_args()

    try:
        cnc = serial.Serial(args.port, args.baudrate, timeout=1)
        time.sleep(2)
        print(f"Conectado a {args.port}. Digite 'exit' para sair.\n")

        while True:
            command = input("GRBL> ").strip()
            if command.lower() == 'exit':
                break

            cnc.write(f"{command}\n".encode())
            while True:
                response = cnc.readline().decode().strip()
                if not response:
                    break
                print(f"Resposta: {response}")

    except Exception as e:
        print(f"Erro: {e}")
    finally:
        if 'cnc' in locals() and cnc.is_open:
            cnc.close()
            print("\nConexão encerrada.")

if __name__ == "__main__":
    main()

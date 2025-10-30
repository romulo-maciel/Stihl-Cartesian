#!/usr/bin/env python3
import serial
import time

SERIAL_PORT = '/dev/grbl'
BAUD_RATE = 115200
TIMEOUT = 1  # segundos


def send_command(ser, cmd, wait_ok=True):
    """Envia comando G-code ao GRBL e opcionalmente aguarda resposta 'ok'."""
    ser.write((cmd + '\n').encode('utf-8'))
    ser.flush()
    if wait_ok:
        wait_for_ok(ser)


def wait_for_ok(ser):
    """Lê linhas da serial até receber 'ok' ou erro."""
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        print('<', line)
        lower = line.lower()
        if 'ok' in lower:
            return
        if 'error' in lower:
            raise RuntimeError(f'Erro do GRBL: {line}')


def home_and_zero(port=SERIAL_PORT, baud=BAUD_RATE):
    """Executa ciclo de homing ($H) e define a posição atual como (0,0)."""
    with serial.Serial(port, baud, timeout=TIMEOUT) as ser:
        send_command(ser, '\r', wait_ok=False)
        time.sleep(2)
        ser.reset_input_buffer()

        print('Iniciando homing...')
        send_command(ser, '$H')

        print('Aguardando fim do homing...')
        time.sleep(1)

        print('Definindo origem em (0,0)')
        send_command(ser, 'G92 X0 Y0')
        print('Origem definida. CNC em (0,0).')


if __name__ == '__main__':
    try:
        home_and_zero()
    except Exception as e:
        print(f'Falha ao executar homing: {e}')

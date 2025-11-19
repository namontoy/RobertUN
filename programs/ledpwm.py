#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import sys

import board
import busio
from adafruit_pca9685 import PCA9685

HOST = "127.0.0.1"
PORT = 8888

def duty_percentage(value: int) -> int:
    """
    Convierte un duty en % (0..100) al valor de 16 bits (0..65535) del PCA9685.
    Se invierte la lógica porque el optoacoplador invierte la salida.
    """
    value = max(0, min(100, int(value)))
    return 65535 - int(round((value / 100.0) * 65535))

# ------------------ Inicialización de hardware ------------------
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 60  # Ajusta según tu carga (servos/LEDs de baja frecuencia)
    PWM_OFF = 65535     # Apagado "lógico" con tu electrónica invertida
except Exception as e:
    print(f"[FATAL HW] No se pudo inicializar el PCA9685: {e}")
    sys.exit(1)

# ------------------ Utilidades de PWM ------------------
def apagar_todo_pwm() -> bool:
    """Apaga los canales 0,1,2 (estado seguro)."""
    try:
        for i in range(3):
            pca.channels[i].duty_cycle = PWM_OFF
        print("[HW] Todos los canales PWM (0,1,2) apagados.")
        return True
    except Exception as e:
        print(f"[HW] Error apagando PWM: {e}")
        return False

def set_pwm_canal(canal: int, duty_pct: int = 50) -> None:
    """
    Enciende 'canal' con 'duty_pct' (0..100) y apaga los otros (0..2).
    """
    if canal not in (0, 1, 2):
        raise ValueError("canal fuera de rango (0,1,2)")
    duty_val = duty_percentage(duty_pct)
    for i in range(3):
        pca.channels[i].duty_cycle = duty_val if i == canal else PWM_OFF

def set_pwm_triplet(dL: int, dC: int, dR: int) -> None:
    """
    Ajusta los tres canales (0:izq, 1:centro, 2:der) con duty en % (0..100).
    """
    dL = max(0, min(100, int(dL)))
    dC = max(0, min(100, int(dC)))
    dR = max(0, min(100, int(dR)))
    vals = [duty_percentage(dL), duty_percentage(dC), duty_percentage(dR)]
    for i in range(3):
        pca.channels[i].duty_cycle = vals[i]

# ------------------ Servidor TCP ------------------
def iniciar_servidor():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen()
            print(f"[SERVIDOR] Escuchando en {HOST}:{PORT}...")

            while True:
                conn, addr = s.accept()
                print(f"[SERVIDOR] Conexión de {addr}")
                with conn:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            apagar_todo_pwm()
                            continue

                        comando = data.decode("utf-8", errors="ignore").strip()

                        if comando.startswith("SET_PWM_ALL:"):
                            try:
                                _, payload = comando.split(":", 1)
                                dL_txt, dC_txt, dR_txt = [x.strip() for x in payload.split(":")]
                                set_pwm_triplet(int(dL_txt), int(dC_txt), int(dR_txt))
                                conn.sendall(b"OK")
                            except Exception as e:
                                print(f"[SERVIDOR] Error SET_PWM_ALL: {e}")
                                conn.sendall(b"ERROR")

                        elif comando.startswith("SET_PWM:"):
                            try:
                                parts = comando.split(":")
                                if len(parts) == 2:
                                    canal = int(parts[1].strip())
                                    set_pwm_canal(canal)  # duty por defecto 50%
                                elif len(parts) == 3:
                                    canal = int(parts[1].strip())
                                    duty = int(parts[2].strip())
                                    set_pwm_canal(canal, duty)
                                else:
                                    raise ValueError("Formato SET_PWM inválido")
                                conn.sendall(b"OK")
                            except Exception as e:
                                print(f"[SERVIDOR] Error procesando SET_PWM: {e}")
                                conn.sendall(b"ERROR")

                        else:
                            conn.sendall(b"ERROR_CMD")

                    except Exception as e:
                        print(f"[SERVIDOR] Excepción manejando conexión: {e}")
                        try:
                            conn.sendall(b"ERROR")
                        except:
                            pass

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C recibido. Apagando PWM y saliendo...")
        apagar_todo_pwm()
    except Exception as e:
        print(f"[FATAL] Servidor falló: {e}")
        apagar_todo_pwm()
        sys.exit(1)

if __name__ == "__main__":
    apagar_todo_pwm()  # seguridad al inicio
    iniciar_servidor()

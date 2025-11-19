import socket
import sys

import board
import busio
from adafruit_pca9685 import PCA9685

HOST = "127.0.0.1"
PORT = 8888

def duty_percentage(value): #Ingresa un porcentaje y lo convierte en valores válidos
    return 65535-int(round(float(value/100) * 65535)) #Resta el valor porque el optoacoplador hace inversión

# Configuramos PCA9685
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 60
    PWM_ON = duty_percentage(50)  # 50% duty
    PWM_OFF = 65535
except Exception as e:
    print(f"[FATAL HW] No se pudo inicializar el PCA9685: {e}")
    sys.exit(1)

def apagar_todo_pwm():
    for i in range(3):  # canales 0,1,2
        pca.channels[i].duty_cycle = PWM_OFF
    print("[HW] Todos los canales PWM (0,1,2) apagados.")
    return True

def set_pwm_canal(canal: int):
    """Enciende 'canal' y apaga los otros (0..2)."""
    if canal not in (0, 1, 2):
        raise ValueError("canal fuera de rango")
    for i in range(3):
        if i == canal:
            pca.channels[i].duty_cycle = PWM_ON  
        else:
            pca.channels[i].duty_cycle = PWM_OFF

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
                        if comando.startswith("SET_PWM:"):
                            canal_txt = comando.split(":", 1)[1].strip()
                            try:
                                canal = int(canal_txt)
                                set_pwm_canal(canal)
                                conn.sendall(b"OK")
                            except Exception as e:
                                print(f"[SERVIDOR] Error procesando canal: {e}")
                                conn.sendall(b"ERROR")
                        else:
                            conn.sendall(b"ERROR_CMD")
                    except Exception as e:
                        print(f"[SERVIDOR] Excepcion: {e}")
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

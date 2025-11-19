# servo_server.py
import socket
import sys
from adafruit_servokit import ServoKit
from inverted_pwm import InvertedServo  # módulo auxiliar

HOST = "127.0.0.1"
PORT = 8888

# =========================
# CONFIGURACIÓN DE HARDWARE
# =========================
try:
    kit = ServoKit(channels=16)

    # Canal 4 → Y
    # Canal 5 → X
    servo_y = InvertedServo(kit.servo[4], kit._pca.channels[4])
    servo_x = InvertedServo(kit.servo[5], kit._pca.channels[5])

    kit.servo[4].set_pulse_width_range(500, 2500)
    kit.servo[5].set_pulse_width_range(500, 2500)

    print("[HW] Servos inicializados correctamente:")
    print("     Canal 4 → Eje Y (PWM invertido)")
    print("     Canal 5 → Eje X (PWM invertido)")
except Exception as e:
    print(f"[FATAL HW] No se pudo inicializar el PCA9685/ServoKit: {e}")
    sys.exit(1)


# =========================
# FUNCIONES DE CONTROL
# =========================
def centrar_servos():
    """Coloca ambos servos en posición neutral (90°)."""
    servo_x.angle = 90
    servo_y.angle = 90
    print("[HW] Servos centrados (posición neutral).")


def mover_servo(canal: str, angulo: float):
    """
    Mueve el servo X o Y al ángulo indicado.
    """
    if canal not in ("X", "Y"):
        raise ValueError("Canal inválido. Usa 'X' o 'Y'.")
    if not 0 <= angulo <= 180:
        raise ValueError("Ángulo fuera de rango (0–180).")

    if canal == "X":
        servo_x.angle = angulo
        print(f"[HW] Servo X (canal 5) movido a {angulo:.1f}°")
    else:
        servo_y.angle = angulo
        print(f"[HW] Servo Y (canal 4) movido a {angulo:.1f}°")


# =========================
# SERVIDOR TCP PRINCIPAL
# =========================
def iniciar_servidor():
    """Servidor TCP que recibe comandos para controlar los servos."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen()
            print(f"[SERVIDOR] Escuchando en {HOST}:{PORT}...")

            while True:
                conn, addr = s.accept()
                print(f"[SERVIDOR] Conexión desde {addr}")
                with conn:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            centrar_servos()
                            continue

                        comando = data.decode("utf-8", errors="ignore").strip()
                        if comando.startswith("SET_SERVO:"):
                            # Ejemplo: SET_SERVO:X:120
                            partes = comando.split(":")
                            if len(partes) == 3:
                                canal = partes[1].strip().upper()
                                angulo = float(partes[2])
                                mover_servo(canal, angulo)
                                conn.sendall(b"OK")
                            else:
                                conn.sendall(b"ERROR_FORMAT")
                        else:
                            conn.sendall(b"ERROR_CMD")
                    except Exception as e:
                        print(f"[SERVIDOR] Error: {e}")
                        try:
                            conn.sendall(b"ERROR")
                        except:
                            pass
    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C recibido. Centrando servos y saliendo...")
        centrar_servos()
    except Exception as e:
        print(f"[FATAL] Servidor falló: {e}")
        centrar_servos()
        sys.exit(1)

if __name__ == "__main__":
    centrar_servos()
    iniciar_servidor()

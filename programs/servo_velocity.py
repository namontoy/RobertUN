# servo_server.py
import socket
import sys
from adafruit_servokit import ServoKit
from inverted_pwm import InvertedServo  # módulo auxiliar

HOST = "127.0.0.1"
PORT = 8888
VERBOSE = False  # pon True si quieres logs de cada movimiento

# ============== HW =================
try:
    kit = ServoKit(channels=16)
    # Canal 4 → Y ; Canal 5 → X
    servo_y = InvertedServo(kit.servo[4], kit._pca.channels[4])
    servo_x = InvertedServo(kit.servo[5], kit._pca.channels[5])
    kit.servo[4].set_pulse_width_range(500, 2500)
    kit.servo[5].set_pulse_width_range(500, 2500)
except Exception as e:
    print(f"[FATAL HW] No se pudo inicializar PCA9685/ServoKit: {e}")
    sys.exit(1)

def clamp_angle(a):  # seguridad extra (0–180 por si reajustas rangos)
    return max(0.0, min(180.0, float(a)))

def centrar_servos():
    servo_x.angle = 90
    servo_y.angle = 90
    if VERBOSE: print("[HW] Centrado X=90, Y=90")

def mover_servo(canal: str, angulo: float):
    angulo = clamp_angle(angulo)
    if canal == "X":
        servo_x.angle = angulo
        if VERBOSE: print(f"[HW] X → {angulo:.1f}°")
    elif canal == "Y":
        servo_y.angle = angulo
        if VERBOSE: print(f"[HW] Y → {angulo:.1f}°")
    else:
        raise ValueError("Canal inválido (usa X o Y)")

# ============== Servidor TCP (persistente) ==============
def iniciar_servidor():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen()
            print(f"[SERVIDOR] Escuchando en {HOST}:{PORT} ...")
            while True:
                conn, addr = s.accept()
                if VERBOSE: print(f"[SERVIDOR] Conexión desde {addr}")
                with conn:
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    buf = b""
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            break
                        buf += data
                        # Protocolo por líneas: cada comando termina en '\n'
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            cmd = line.decode("utf-8", errors="ignore").strip()
                            if not cmd:
                                continue
                            try:
                                if cmd == "PING":
                                    conn.sendall(b"PONG\n")
                                elif cmd.startswith("SET_SERVO:"):
                                    # Ej: SET_SERVO:X:120.5
                                    _, canal, aval = cmd.split(":")
                                    mover_servo(canal.strip().upper(), float(aval))
                                    conn.sendall(b"OK\n")
                                elif cmd == "CENTER":
                                    centrar_servos()
                                    conn.sendall(b"OK\n")
                                else:
                                    conn.sendall(b"ERROR_CMD\n")
                            except Exception as e:
                                if VERBOSE: print(f"[SERVIDOR] Error cmd: {e}")
                                conn.sendall(b"ERROR\n")
    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C → centrando y saliendo...")
        centrar_servos()
    except Exception as e:
        print(f"[FATAL] Servidor falló: {e}")
        centrar_servos()
        sys.exit(1)

if __name__ == "__main__":
    centrar_servos()
    iniciar_servidor()

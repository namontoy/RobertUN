import cv2
import numpy as np
import socket
import time

# ================== RED / SERVOS ==================
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8888
SOCK_TIMEOUT = 0.05  # respuestas rápidas
COMMAND_RATE_LIMIT = 0.02  # seg mínimos entre comandos (protege bus)
ANGLE_DEADBAND = 0.4       # ° mínimo para mandar
MAX_STEP_DEG = 3.0         # slew-rate por actualización

MIN_ANGLE = 45.0
MAX_ANGLE = 135.0
CURRENT_ANGLE_X = 90.0
CURRENT_ANGLE_Y = 90.0

class ServoClient:
    def __init__(self, host, port, timeout=0.05):
        self.host, self.port = host, port
        self.timeout = timeout
        self.sock = None
        self._last_cmd_ts = 0.0

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect((self.host, self.port))

    def close(self):
        try:
            if self.sock: self.sock.close()
        finally:
            self.sock = None

    def _send_line(self, line: str) -> bool:
        # rate limiting
        now = time.perf_counter()
        if now - self._last_cmd_ts < COMMAND_RATE_LIMIT:
            return True  # “OK” virtual: sólo limita frecuencia
        try:
            self.sock.sendall((line + "\n").encode("utf-8"))
            resp = self.sock.recv(16)  # "OK\n"/"ERROR\n"
            self._last_cmd_ts = now
            return resp.startswith(b"OK") or resp.startswith(b"PONG")
        except Exception:
            return False

    def set_servo(self, axis: str, angle: float) -> bool:
        return self._send_line(f"SET_SERVO:{axis}:{angle:.1f}")

    def center(self):
        return self._send_line("CENTER")

# ================== VISIÓN ==================
# Procesa a menor resolución para acelerar (escalado interno)
PROC_W, PROC_H = 640, 360
MORPH_K = 5
MIN_AREA_RATIO = 0.002  # relativo a PROC_W*PROC_H

# ================== PID ==================
# Ganancias base calibradas para 1280 px; se escalan con el ancho de proceso
BASE_KP_X, BASE_KI_X, BASE_KD_X = 0.0030, 0.0002, 0.0020
BASE_KP_Y, BASE_KI_Y, BASE_KD_Y = 0.0020, 0.0002, 0.0020
DERIV_FILTER = 0.85   # filtro exponencial para derivada
I_CLAMP = 2000.0      # límite anti-windup del integrador (en “px·s”)

def gstreamer_pipeline(sensor_id, capture_width, capture_height,
                       display_width, display_height, framerate, flipmethod):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=%d, height=%d, framerate=%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=%d, height=%d, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink drop=true sync=false"
        % (sensor_id, capture_width, capture_height, framerate, flipmethod,
           display_width, display_height)
    )

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

# ================== ESCANEO ==================
LOST_TIMEOUT = 0.35  # s sin detección → empezar escaneo
SCAN_DWELL = 0.40    # s de permanencia por punto de escaneo
SCAN_POS_X = [60.0, 90.0, 120.0]   # 3 posiciones X
SCAN_POS_Y = [70.0, 90.0, 110.0]   # 3 posiciones Y (abajo, centro, arriba)

# ================== MAIN ==================
cv2.setUseOptimized(True)

lower_yellow = np.array([20, 100, 100]); upper_yellow = np.array([30, 255, 255])
lower_green  = np.array([85, 100, 100]); upper_green  = np.array([128, 255, 255])
lower_red    = np.array([0, 100, 100]);  upper_red    = np.array([8, 255, 255])
lower_red2   = np.array([175,100,100]);  upper_red2   = np.array([179,255,255])

color_selected = input("Ingrese un color (R/G/Y): ").strip().upper()
print("Presione 'q' para salir.")

pipeline = gstreamer_pipeline(
    sensor_id=0, capture_width=1280, capture_height=720,
    display_width=1280, display_height=720, framerate=60, flipmethod=0
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise RuntimeError("ERROR: No se pudo abrir la cámara con GStreamer.")

# Conectar al servidor y centrar
servo = ServoClient(SERVER_HOST, SERVER_PORT, timeout=SOCK_TIMEOUT)
servo.connect()
servo.center()

# PID state
scale_k = 1280.0 / PROC_W
KP_X, KI_X, KD_X = BASE_KP_X*scale_k, BASE_KI_X*scale_k, BASE_KD_X*scale_k
KP_Y, KI_Y, KD_Y = BASE_KP_Y*scale_k, BASE_KI_Y*scale_k, BASE_KD_Y*scale_k

integral_x = 0.0
integral_y = 0.0
last_error_x = 0.0
last_error_y = 0.0
d_x = 0.0
d_y = 0.0
last_ts = time.perf_counter()

last_seen_ts = last_ts
scanning = False
scan_ix = 0  # 0..(len(SCAN_POS_X)*len(SCAN_POS_Y)-1)
last_scan_ts = last_ts

kernel = np.ones((MORPH_K, MORPH_K), np.uint8)

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("ERROR: Fallo al leer frame.")
            break

        # Salida
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

        # Redimensionar para procesar rápido
        proc = cv2.resize(frame, (PROC_W, PROC_H), interpolation=cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)

        if color_selected == "R":
            mask = cv2.inRange(hsv, lower_red, upper_red) + cv2.inRange(hsv, lower_red2, upper_red2)
        elif color_selected == "G":
            mask = cv2.inRange(hsv, lower_green, upper_green)
        else:
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Morfología para limpiar ruido
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        center_x = PROC_W * 0.5
        center_y = PROC_H * 0.5

        best_x = best_y = -1
        max_area = 0.0
        area_min = MIN_AREA_RATIO * (PROC_W * PROC_H)

        for c in contours:
            area = cv2.contourArea(c)
            if area < area_min:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            if area > max_area:
                max_area = area
                best_x, best_y = x, y

        now = time.perf_counter()
        dt = max(1e-3, now - last_ts)
        last_ts = now

        if max_area > 0:
            # Detección válida → salir del escaneo si estaba activo
            scanning = False
            last_seen_ts = now

            # Errores (px)
            error_x = (best_x - center_x)
            error_y = (center_y - best_y)

            # Integrales (anti-windup: se clampa y sólo se integra si hay target)
            integral_x = clamp(integral_x + error_x * dt, -I_CLAMP, I_CLAMP)
            integral_y = clamp(integral_y + error_y * dt, -I_CLAMP, I_CLAMP)

            # Derivada filtrada (sobre el error)
            raw_dx = (error_x - last_error_x) / dt
            raw_dy = (error_y - last_error_y) / dt
            d_x = DERIV_FILTER * d_x + (1.0 - DERIV_FILTER) * raw_dx
            d_y = DERIV_FILTER * d_y + (1.0 - DERIV_FILTER) * raw_dy
            last_error_x, last_error_y = error_x, error_y

            # Control PID → cambio en ángulo (°)
            change_x = KP_X * error_x + KI_X * integral_x + KD_X * d_x
            change_y = KP_Y * error_y + KI_Y * integral_y + KD_Y * d_y

            # Target absolutos (recordatorio: signo invertido por tu cinemática)
            target_x = CURRENT_ANGLE_X - change_x
            target_y = CURRENT_ANGLE_Y - change_y

            # Saturación
            target_x = clamp(target_x, MIN_ANGLE, MAX_ANGLE)
            target_y = clamp(target_y, MIN_ANGLE, MAX_ANGLE)

            # Slew-rate: no saltar más de MAX_STEP_DEG por update
            step_x = clamp(target_x - CURRENT_ANGLE_X, -MAX_STEP_DEG, MAX_STEP_DEG)
            step_y = clamp(target_y - CURRENT_ANGLE_Y, -MAX_STEP_DEG, MAX_STEP_DEG)
            target_x = CURRENT_ANGLE_X + step_x
            target_y = CURRENT_ANGLE_Y + step_y

            # Deadband y envío
            if abs(target_x - CURRENT_ANGLE_X) >= ANGLE_DEADBAND:
                if servo.set_servo("X", target_x):
                    CURRENT_ANGLE_X = target_x
            if abs(target_y - CURRENT_ANGLE_Y) >= ANGLE_DEADBAND:
                if servo.set_servo("Y", target_y):
                    CURRENT_ANGLE_Y = target_y

            # Overlay simple
            cv2.circle(proc, (int(center_x), int(center_y)), 8, (255, 255, 255), 2)
            cv2.circle(proc, (int(best_x), int(best_y)), 5, (0, 0, 255), -1)
            cv2.line(proc, (int(best_x), int(best_y)),
                     (int(center_x), int(center_y)), (0, 0, 255), 1)
            txt = f"PID | ex:{error_x:.0f} ey:{error_y:.0f} | X:{CURRENT_ANGLE_X:.1f} Y:{CURRENT_ANGLE_Y:.1f}"
            cv2.putText(proc, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)

        else:
            # No detección
            cv2.putText(proc, "Sin objetivo", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

            # congela integradores para no acumular error “fantasma”
            last_error_x = last_error_y = 0.0

            if not scanning and (now - last_seen_ts) >= LOST_TIMEOUT:
                scanning = True
                scan_ix = 0
                last_scan_ts = now

            if scanning and (now - last_scan_ts) >= SCAN_DWELL:
                last_scan_ts = now
                # Secuencia: para cada X en SCAN_POS_X, recorrer Y en [abajo, centro, arriba]
                num_x = len(SCAN_POS_X)
                num_y = len(SCAN_POS_Y)
                total = num_x * num_y
                ix_x = (scan_ix // num_y) % num_x
                ix_y = scan_ix % num_y
                target_x = clamp(SCAN_POS_X[ix_x], MIN_ANGLE, MAX_ANGLE)
                target_y = clamp(SCAN_POS_Y[ix_y], MIN_ANGLE, MAX_ANGLE)

                # Mover (con deadband/slew también)
                if abs(target_x - CURRENT_ANGLE_X) >= ANGLE_DEADBAND:
                    step_x = clamp(target_x - CURRENT_ANGLE_X, -MAX_STEP_DEG, MAX_STEP_DEG)
                    if servo.set_servo("X", CURRENT_ANGLE_X + step_x):
                        CURRENT_ANGLE_X += step_x
                if abs(target_y - CURRENT_ANGLE_Y) >= ANGLE_DEADBAND:
                    step_y = clamp(target_y - CURRENT_ANGLE_Y, -MAX_STEP_DEG, MAX_STEP_DEG)
                    if servo.set_servo("Y", CURRENT_ANGLE_Y + step_y):
                        CURRENT_ANGLE_Y += step_y

                scan_ix = (scan_ix + 1) % total  # siguiente punto

        cv2.imshow("Tracking (proc)", proc)

finally:
    try:
        servo.close()
    except:
        pass
    cap.release()
    cv2.destroyAllWindows()

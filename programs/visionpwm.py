import cv2
import numpy as np
import socket
import time

SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8888
TIMEOUT = 0.5

# Envío de comandos al servidor
def enviar_comando_control(comando: str) -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(TIMEOUT)
            s.connect((SERVER_HOST, SERVER_PORT))
            s.sendall(comando.encode("utf-8"))
            resp = s.recv(1024)
            return resp.startswith(b"OK")
    except Exception:
        return False

# Pipeline para Jetson/CSI. Si no usas Jetson, cambia a cap = cv2.VideoCapture(0)
def gstreamer_pipeline(sensor_id, capture_width, capture_height, display_width, display_height, framerate, flipmethod):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=%d, height=%d, framerate=%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=%d, height=%d, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink drop=true sync=false"
        % (sensor_id, capture_width, capture_height, framerate, flipmethod, display_width, display_height)
    )

# ------------------ Abrir cámara ------------------
USE_GSTREAMER = True  # pon False para usar webcam estándar
if USE_GSTREAMER:
    pipeline = gstreamer_pipeline(
        sensor_id=0, capture_width=1280, capture_height=720,
        display_width=1280, display_height=720, framerate=60, flipmethod=0
    )
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
else:
    cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise RuntimeError("ERROR: No se pudo abrir la cámara.")

# ------------------ Rangos HSV ------------------
lower_yellow = np.array([20, 100, 100]); upper_yellow = np.array([30, 255, 255])
lower_green  = np.array([ 85, 100, 100]); upper_green  = np.array([128, 255, 255])
lower_red1   = np.array([  0, 100, 100]); upper_red1   = np.array([  8, 255, 255])
lower_red2   = np.array([175, 100, 100]); upper_red2   = np.array([179, 255, 255])

# ------------------ Selección de color ------------------
color_selected = input("Ingrese un color (R/G/Y): ").strip().upper()
if color_selected not in ("R", "G", "Y"):
    print("Color no reconocido, se usará Amarillo (Y).")
    color_selected = "Y"

AREA_MIN = 2000        # área mínima para considerar contorno (ruido)
MIN_DUTY = 0           # mínimo duty para no quedar en 0 absoluto (p.ej., 10)
SEND_HZ  = 15          # tope de frecuencia de envío de comandos
send_period = 1.0 / max(1, SEND_HZ)
last_send_t = 0.0
last_triplet = (-1, -1, -1)  # para no spamear lo mismo

def weights_from_x(best_x: int, width: int):
    """
    Calcula pesos lineales (triángulos) para Izq/Centro/Der en función de la
    posición horizontal. Devuelve (wL, wC, wR) en [0..1], con el máximo normalizado a 1.
    """
    if width <= 1:
        return (1.0, 0.0, 0.0)
    t = best_x / (width - 1.0)  # normalizado 0..1

    wL = max(0.0, 1.0 - 2.0*t)               # pico en 0
    wC = max(0.0, 1.0 - 2.0*abs(t - 0.5))    # pico en 0.5
    wR = max(0.0, 2.0*t - 1.0)               # pico en 1.0

    m = max(wL, wC, wR, 1e-6)                # normaliza para que el máximo sea 1
    return (wL/m, wC/m, wR/m)

def weights_to_duty(wL, wC, wR, min_duty=0):
    """
    Convierte pesos [0..1] a % duty [0..100], aplicando un piso opcional.
    """
    if min_duty <= 0:
        return (int(round(100*wL)), int(round(100*wC)), int(round(100*wR)))
    scale = 100 - min_duty
    lift = lambda w: min_duty + int(round(scale * w))
    return (lift(wL), lift(wC), lift(wR))

print("[INFO] Presiona 'q' para salir.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Fallo al leer frame.")
            break

        # Salida con 'q'
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

        # --- Preprocesamiento ---      
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # --- Máscara por color ---
        if color_selected == "R":
            mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        elif color_selected == "G":
            mask = cv2.inRange(hsv, lower_green, upper_green)
        else:  # Y
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Limpieza morfológica opcional
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # --- Contornos y selección del mayor ---
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        best_x = -1
        max_area = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area < AREA_MIN:
                continue
            M = cv2.moments(c)
            m00 = M["m00"] if M["m00"] != 0 else 1
            cx = int(M["m10"] / m00)
            cy = int(M["m01"] / m00)

            if area > max_area:
                max_area = area
                best_x = cx

            x_, y_, w_, h_ = cv2.boundingRect(c)
            cv2.rectangle(frame, (x_, y_), (x_+w_, y_+h_), (0, 255, 0), 2)
            cv2.putText(frame, f"{cx},{cy}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

        # --- Si hay objeto, calcular tripleta y enviar ---
        if max_area > 0 and best_x >= 0:
            fw = frame.shape[1]
            wL, wC, wR = weights_from_x(best_x, fw)
            dL, dC, dR = weights_to_duty(wL, wC, wR, min_duty=MIN_DUTY)

            # Throttle/envío solo si cambió o pasó tiempo
            now = time.time()
            triplet = (dL, dC, dR)
            changed = triplet != last_triplet
            if changed and (now - last_send_t) >= send_period:
                cmd = f"SET_PWM_ALL:{dL}:{dC}:{dR}"
                ok = enviar_comando_control(cmd)
                if ok:
                    last_triplet = triplet
                    last_send_t = now

            # Overlay de debug
            cv2.putText(frame, f"duty L/C/R: {dL}/{dC}/{dR}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

            # Líneas guía de terciles
            t1 = fw // 3
            t2 = 2 * fw // 3
            cv2.line(frame, (t1, 0), (t1, frame.shape[0]), (255, 255, 255), 1)
            cv2.line(frame, (t2, 0), (t2, frame.shape[0]), (255, 255, 255), 1)

        cv2.imshow("Camera", frame)

finally:
    cap.release()
    cv2.destroyAllWindows()

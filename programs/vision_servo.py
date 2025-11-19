import cv2
import numpy as np
import socket

SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8888
TIMEOUT = 0.5

def enviar_comando_control(comando: str) -> bool:
    """Envía comando al servidor y devuelve True si recibe 'OK'."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(TIMEOUT)
            s.connect((SERVER_HOST, SERVER_PORT))
            s.sendall(comando.encode("utf-8"))
            resp = s.recv(1024)
            return resp.startswith(b"OK")
    except Exception:
        return False

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

# Configura cámara
pipeline = gstreamer_pipeline(
    sensor_id=0, capture_width=1280, capture_height=720,
    display_width=1280, display_height=720, framerate=60, flipmethod=0
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise RuntimeError("ERROR: No se pudo abrir la cámara con GStreamer.")

# Rangos HSV de colores
lower_yellow = np.array([20, 100, 100]); upper_yellow = np.array([30, 255, 255])
lower_green  = np.array([85, 100, 100]); upper_green  = np.array([128, 255, 255])
lower_red    = np.array([0, 100, 100]);  upper_red    = np.array([8, 255, 255])
lower_red2   = np.array([175,100,100]);  upper_red2   = np.array([179,255,255])

color_selected = input("Ingrese un color (R/G/Y): ").strip().upper()
print("Presione 'q' para salir.")

# Últimos sectores para evitar spam de comandos
last_sector_x = None
last_sector_y = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("ERROR: Fallo al leer frame.")
        break

    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Máscara por color
    if color_selected == "R":
        mask = cv2.inRange(hsv, lower_red, upper_red) + cv2.inRange(hsv, lower_red2, upper_red2)
    elif color_selected == "G":
        mask = cv2.inRange(hsv, lower_green, upper_green)
    else:
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    best_x, best_y = -1, -1
    max_area = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < 2000:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])

        if area > max_area:
            max_area = area
            best_x, best_y = x, y

        x_, y_, w_, h_ = cv2.boundingRect(c)
        cv2.rectangle(frame, (x_, y_), (x_ + w_, y_ + h_), (0, 255, 0), 2)
        cv2.putText(frame, f"{x},{y}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

    if max_area > 0 and best_x >= 0 and best_y >= 0:
        fw = frame.shape[1]
        fh = frame.shape[0]

        # División de zonas horizontales y verticales
        zone_x = fw / 3.0
        zone_y = fh / 3.0

        # Determinar sector horizontal (izq/centro/der)
        if best_x < zone_x:
            sector_x = "LEFT"
            angle_x = 120
        elif best_x > 2 * zone_x:
            sector_x = "RIGHT"
            angle_x = 45
        else:
            sector_x = "CENTER"
            angle_x = 90

        # Determinar sector vertical (arriba/centro/abajo)
        if best_y < zone_y:
            sector_y = "TOP"
            angle_y = 45
        elif best_y > 2 * zone_y:
            sector_y = "BOTTOM"
            angle_y = 120
        else:
            sector_y = "CENTER"
            angle_y = 90

        # Enviar comandos solo si el sector cambió
        if sector_x != last_sector_x:
            enviar_comando_control(f"SET_SERVO:X:{angle_x}")
            last_sector_x = sector_x
        if sector_y != last_sector_y:
            enviar_comando_control(f"SET_SERVO:Y:{angle_y}")
            last_sector_y = sector_y

        # Dibujar líneas de zonas
        cv2.line(frame, (int(zone_x), 0), (int(zone_x), fh), (255, 0, 0), 1)
        cv2.line(frame, (int(2 * zone_x), 0), (int(2 * zone_x), fh), (255, 0, 0), 1)
        cv2.line(frame, (0, int(zone_y)), (fw, int(zone_y)), (255, 0, 0), 1)
        cv2.line(frame, (0, int(2 * zone_y)), (fw, int(2 * zone_y)), (255, 0, 0), 1)

        # Mostrar texto de zona
        cv2.putText(frame, f"X:{sector_x} Y:{sector_y}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

    cv2.imshow("Tracking", frame)

cap.release()
cv2.destroyAllWindows()


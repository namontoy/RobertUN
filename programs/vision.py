import cv2
import numpy as np
import socket

SERVER_HOST = "127.0.0.1"  
SERVER_PORT = 8888
TIMEOUT = 0.5

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

pipeline = gstreamer_pipeline(
    sensor_id=0, capture_width=1280, capture_height=720,
    display_width=1280, display_height=720, framerate=60, flipmethod=0
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise RuntimeError("ERROR: No se pudo abrir la cámara con GStreamer.")

# Rangos HSV
lower_yellow = np.array([20, 100, 100]); upper_yellow = np.array([30, 255, 255])
lower_green  = np.array([85, 100, 100]); upper_green  = np.array([128, 255, 255])
lower_red    = np.array([0, 100, 100]);  upper_red    = np.array([8, 255, 255])
lower_red2   = np.array([175,100,100]);  upper_red2   = np.array([179,255,255])

color_selected = input("Ingrese un color (R/G/Y): ").strip().upper()

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
    else:  # "Y" por defecto
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    best_x = -1
    max_area = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < 2000:
            continue
        M = cv2.moments(c)
        m00 = M["m00"] if M["m00"] != 0 else 1
        x = int(M["m10"] / m00); y = int(M["m01"] / m00)

        if area > max_area:
            max_area = area
            best_x = x

        x_, y_, w_, h_ = cv2.boundingRect(c)
        cv2.rectangle(frame, (x_, y_), (x_+w_, y_+h_), (0,255,0), 2)
        cv2.putText(frame, f"{x},{y}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

    # Solo enviar si realmente detectamos algo grande
    if max_area > 0 and best_x >= 0:
        fw = frame.shape[1]
        zone = fw / 3.0
        if best_x < zone:
            cmd = "SET_PWM:0"
        elif best_x > 2*zone:
            cmd = "SET_PWM:2"
        else:
            cmd = "SET_PWM:1"
        ok = enviar_comando_control(cmd)
        if not ok:
            # No interrumpimos el loop
            pass

    cv2.imshow("Camera", frame)

cap.release()
cv2.destroyAllWindows()

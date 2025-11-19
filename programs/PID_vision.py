import cv2
import numpy as np
import socket
import time # Importar time para el control

SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8888
TIMEOUT = 0.1

# --- Configuración del Control P ---
# Ganancias Proporcionales (ajustar según el robot)
KP_X = 0.003
KP_Y = 0.002

# Rango de ángulo de los servos
MIN_ANGLE = 45.0
MAX_ANGLE = 135.0
# Posición inicial de los servos (centro)
CURRENT_ANGLE_X = 90.0
CURRENT_ANGLE_Y = 90.0
# Ángulo mínimo para considerar movimiento
ANGLE_DEADBAND = 0.5

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
        # print(f"Error al enviar comando: {e}") # Descomentar para debug
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
# El pipeline se mantiene igual
pipeline = gstreamer_pipeline(
    sensor_id=0, capture_width=1280, capture_height=720,
    display_width=1280, display_height=720, framerate=60, flipmethod=0
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise RuntimeError("ERROR: No se pudo abrir la cámara con GStreamer.")

# Rangos HSV de colores (se mantienen igual)
lower_yellow = np.array([20, 100, 100]); upper_yellow = np.array([30, 255, 255])
lower_green  = np.array([85, 100, 100]); upper_green  = np.array([128, 255, 255])
lower_red    = np.array([0, 100, 100]);  upper_red    = np.array([8, 255, 255])
lower_red2   = np.array([175,100,100]);  upper_red2   = np.array([179,255,255])

color_selected = input("Ingrese un color (R/G/Y): ").strip().upper()
print("Presione 'q' para salir.")



# Se envía la posición inicial al servidor al inicio.
enviar_comando_control(f"SET_SERVO:X:{CURRENT_ANGLE_X}")
enviar_comando_control(f"SET_SERVO:Y:{CURRENT_ANGLE_Y}")


while True:
    ret, frame = cap.read()
    if not ret:
        print("ERROR: Fallo al leer frame.")
        break

    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        break

    fw = frame.shape[1]
    fh = frame.shape[0]
    center_x = fw / 2.0
    center_y = fh / 2.0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Máscara por color (se mantiene igual)
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

        # Dibujar rectángulo y centroide (se mantiene igual)
        x_, y_, w_, h_ = cv2.boundingRect(c)
        cv2.rectangle(frame, (x_, y_), (x_ + w_, y_ + h_), (0, 255, 0), 2)
        cv2.putText(frame, f"Area:{area:.0f}", (x_, y_ - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)


    if max_area > 0 and best_x >= 0 and best_y >= 0:
        # --- CÁLCULO DEL ERROR Y CONTROL P (PID) ---

        # 1. Error: Distancia desde el centroide al centro del frame
        error_x = best_x - center_x
        error_y = center_y - best_y 

        # 2. Término Proporcional (P)
        # Ajuste = Error * Ganancia (Kp)
        change_x = error_x * KP_X
        change_y = error_y * KP_Y
        
        # 3. Términos I y D 
        # # integral_x = integral_x + error_x # TBD
        # # derivative_x = error_x - last_error_x # TBD
        # # change_x = (error_x * KP_X) + (integral_x * KI_X) + (derivative_x * KD_X) # TBD

        # 4. Cálculo del nuevo ángulo objetivo
        target_angle_x = CURRENT_ANGLE_X - change_x # Restar porque un +error_x (derecha) debe reducir el ángulo del servo (mover izq)
        target_angle_y = CURRENT_ANGLE_Y - change_y # Restar porque un -error_y (arriba) debe aumentar el ángulo del servo (mover arriba)

        # 5. Restricción del ángulo (Saturación)
        target_angle_x = np.clip(target_angle_x, MIN_ANGLE, MAX_ANGLE)
        target_angle_y = np.clip(target_angle_y, MIN_ANGLE, MAX_ANGLE)

        # 6. Enviar comandos solo si el cambio es significativo (Deadband)
        if abs(target_angle_x - CURRENT_ANGLE_X) >= ANGLE_DEADBAND:
            if enviar_comando_control(f"SET_SERVO:X:{target_angle_x:.1f}"):
                CURRENT_ANGLE_X = target_angle_x

        if abs(target_angle_y - CURRENT_ANGLE_Y) >= ANGLE_DEADBAND:
            if enviar_comando_control(f"SET_SERVO:Y:{target_angle_y:.1f}"):
                CURRENT_ANGLE_Y = target_angle_y

        # Dibujar centro del frame y error
        cv2.circle(frame, (int(center_x), int(center_y)), 10, (255, 255, 255), 2)
        cv2.line(frame, (best_x, best_y), (int(center_x), int(center_y)), (0, 0, 255), 1)

        # Mostrar texto de estado
        status_text = f"X_Err:{error_x:.1f} Y_Err:{error_y:.1f} | X_Ang:{CURRENT_ANGLE_X:.1f} Y_Ang:{CURRENT_ANGLE_Y:.1f}"
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

    else:
        # Si no hay objeto, puede detener el movimiento o volver al centro.
        # Por ahora, mantendremos la posición, pero puedes agregar aquí la lógica de centrado.
        status_text = "Objeto no detectado. Centrado"
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Centrar Servo X solo si no está ya en 90.0
        if CURRENT_ANGLE_X != 90.0:
            if enviar_comando_control(f"SET_SERVO:X:90.0"):
                CURRENT_ANGLE_X = 90.0

        # Centrar Servo Y solo si no está ya en 90.0
        if CURRENT_ANGLE_Y != 90.0:
            if enviar_comando_control(f"SET_SERVO:Y:90.0"):
                CURRENT_ANGLE_Y = 90.0

    cv2.imshow("Tracking", frame)

cap.release()
cv2.destroyAllWindows()
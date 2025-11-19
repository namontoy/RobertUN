import cv2

def gstreamer_pipeline( sensor_id, capture_width, capture_height, display_width, display_height,
framerate,flipmethod):
    return(
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=%d, height=%d, framerate=%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=%d, height=%d, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink drop=true sync=false"
        %(
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flipmethod,
            display_width,
            display_height,
        )
    )

pipeline = gstreamer_pipeline(sensor_id = 0, capture_width = 1280, capture_height = 720, display_width = 1280,
display_height = 720, framerate = 30, flipmethod = 0,)

capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while True:
    ret, frame = capture.read()
    # roi = frame[500:1000,200:800] #Fila, columna esto es una referencia y todos los ajustes que se hagan en la princiapl aparecerán acá
    #Soluciando lo anterior hacemos una copia
    roi = frame[400:600,600:800].copy()
    roi_Gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi_Gray = cv2.cvtColor(roi_Gray, cv2.COLOR_GRAY2BGR) #Recuperar la información en puntos
    frame[400:600, 600:800] = roi_Gray
    # frame[400:600, 600:800] = [255,255,255] #Convertir directamente una parte de la camara en un recuadro blanco
    if cv2.waitKey(1)==ord('q'):
        break

    cv2.imshow("ROI",roi)
    cv2.imshow("Gray",roi_Gray)
    cv2.moveWindow("Gray",700,0)
    cv2.moveWindow("ROI",0,0) #Ubicar la camara arriba a la izquierda
    cv2.imshow("Camera", frame)

capture.release()
cv2.destroyAllWindows()
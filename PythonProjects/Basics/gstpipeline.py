#Defininimos una función que nos permita crear un pipeline para gstreamer 
#En esta función vamos a defininir los parametros de la cámara 

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

#Configuración del cuadrado 

side = 100 #longitud del lado en pixeles
color = (0, 255,0 ) #BGR 
thickness = 2 #use -1 to fill

while True:
    ret, frame = capture.read()
    if not ret: #Si falla el ret que es una bandera que indica si todo marcha bien
        print("Error al ejecutar la cámara")
        break
    h, w = frame.shape[:2] #shape tiene un array que indica altura, ancho y canales por eso hacemos un slicing
    # para obtener solo altura y ancho (height and width) h coordenada vertical y w coordenada horizontal

    center_x, center_y = w//2 , h//2

    half = side//2

    x1,y1 = center_x-half, center_y-half
    x2,y2 = center_x+half, center_y+half

    cv2.rectangle(frame, (x1,y1), (x2,y2), color, thickness)
    #Parametros son: dónde dibujar (en qué frame), cuáles son los puntos (iniciales y finales), qué color 
    # y por último el grosor 
    cv2.imshow("Preview CSI CAMERA", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'): #Indica qué debo presionar para salir del bucle True y dejar de ejecutar 
        #En este caso la letra q minuscula
        break 

capture.release()
cv2.destroyAllWindows()
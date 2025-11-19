import cv2
import numpy as np


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

#Ventanas y trackbar

capture_window = 'Capture'
countors_window = 'Contours'
cv2.namedWindow(capture_window)
cv2.namedWindow(countors_window)

max_thresh = 255
intial_thresh = 100
cv2.createTrackbar('CannyThresh:', capture_window,intial_thresh,max_thresh, lambda v: None)

while True:
    ret, frame = capture.read()

    if not ret:
        break

    if cv2.waitKey(1)==ord('q'):
        break

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Convierto a grises
    frame_gray = cv2.GaussianBlur(frame,(5,5),1.2) # El segundo parametro es el kernel size 

    #Se aplica canny que se ajusta desde el trackbar

    thresh = cv2.getTrackbarPos('CannyThresh:', capture_window) #El valor que tenga el trackbar

    if thresh <= 0:
        thresh = 1
    canny_output = cv2.Canny(frame_gray, thresh, thresh*2) # Calcula canny desde el valor actual del thresh 
    #umbral bajo hasta un umbral alto, sirve para ajustar la sensibilidad de la detección de bordes


    countours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    """
    La anterior línea extrae los contornos de la imagen binaria producida por canny
    donde los bordes son pixeles blancos 255 sobre un fondo negro 0
    RETR_TREE guarda subcontornos es quién decide la jerarquía
    CHAIN_APROX es el método que se usa para extraer  estos mismos 
    La función extrae un array con cada contorno es decir [contorno1, contorno2], pero
    cada contorno es un tupla de posiciones x y y.

    _ Este simbolo sirve para no guardar algo que regresa la función
    """

    countours_poly = [None] * len(countours) #toma el número de contornos hallados y crea un array
    # [None, None...]
    boundRect = [None] * len(countours)

    #Ambos se usan más adelante para guardar la aproximación y el rectangulo limitador


    for index, value_element in enumerate(countours):
        countours_poly[index] = cv2.approxPolyDP(value_element,3,True) 
        #Función aproxima a poligono con menos vértices y marca 3 debido a que 
        # es la distancia máxima en pixeles entre el contorno original y la aproximación
        #intentando ser preciso, el true es para cerrar el poligono
        boundRect[index] = cv2.boundingRect(countours_poly[index]) 
    #ya con la aproximación obtiene el rectangulo mínimo que lo encierra, 
    # este rectangulo es el que vamos a dibujar

    #LA PARTE A CONTINUACIÓN DIBUJA EL CONTORNO Y LOS RECTÁNGULOS

    drawing = np.zeros((canny_output.shape[0],canny_output.shape[1],3), dtype=np.uint8)

    #Lo anterior crea una imagen completamente negra del tamaño del canny, 3 canales de tipo 8bits (0-255)

    for i in range(len(countours)):
        color = (0,255,0)
        cv2.drawContours(drawing, countours_poly, i, color, 1)
        
        #La anterior función dibuja en drawing, dibuja contorno por contorno por el index, 
        # y elige color y pixel
        pos_x, pos_y, rectangle_width, rectangle_height = boundRect[i] #regresa el rectángulo que se formó
        #posición arriba a la izquierda
        cv2.rectangle(frame,(int(pos_x),int(pos_y)),(int(pos_x+rectangle_width),int(pos_y+rectangle_width)),color, 2)
        #función anterior dibuja, explicada en anteiores códigos de prueba

    cv2.imshow("Camera", frame) #Se visualiza la cámara
    cv2.imshow(countors_window, drawing) #Se visualiza los contornos con su rectángulo

capture.release()
cv2.destroyAllWindows()
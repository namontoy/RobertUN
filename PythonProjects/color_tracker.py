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

#Crea un trackbar para ajustar los colores 


"""
    Espacio de color 
    Hue: 0 a 179
    Saturation: 0 a 255
    Value de 0 a 255
"""


# Definimos los colores:

#Amarillo
lower_yellow = np.array([20,100,100])
upper_yellow = np.array([30,255,255])

#Verde
lower_green = np.array([85,100,100])
upper_green = np.array([128,255,255])

#Rojo tiene 2 rangos

lower_red = np.array([0,100,100])
upper_red = np.array([8,255,255])

lower_red2 = np.array([175,100,100])
upper_red2 = np.array([179,255,255])

#input para pedir el color que desea
color_selected = str(input("Ingrese un color: (RGY)"))

while True:
    ret, frame = capture.read()

    if not ret:
        break

    if cv2.waitKey(1)==ord('Q'):
        break


    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if color_selected == 'R':
        maskR1 = cv2.inRange(frameHSV, lower_red, upper_red)
        maskR2 = cv2.inRange(frameHSV, lower_red2, upper_red2)
        maskR = cv2.add(maskR1,maskR2)
        countours, _ = cv2.findContours(maskR, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    elif color_selected == 'Y':
        maskY = cv2.inRange(frameHSV, lower_yellow, upper_yellow)
        countours, _ = cv2.findContours(maskY, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    elif color_selected == 'G':
        maskG = cv2.inRange(frameHSV, lower_green, upper_green)
        countours, _ = cv2.findContours(maskG, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # maskVisual= cv2.bitwise_and(frame,frame, mask=maskY)


    countours_poly = [None] * len(countours) #toma el número de contornos hallados y crea un array
    # [None, None...]
    boundRect = [None] * len(countours)

    #Ambos se usan más adelante para guardar la aproximación y el rectangulo limitador


    for index, value_element in enumerate(countours):
        countours_poly[index] = cv2.approxPolyDP(value_element,3,True) 
        boundRect[index] = cv2.boundingRect(countours_poly[index])

    drawing = np.zeros((frameHSV.shape[0],frameHSV.shape[1],3), dtype=np.uint8) #Mascara donde dibujaremos contornos

    for i in range(len(countours)):
        area = cv2.contourArea(countours_poly[i])
        color = (0,255,0)
        if area > 2000:
            cv2.drawContours(drawing, countours_poly, i, color, 1)
            pos_x, pos_y, rectangle_width, rectangle_height = boundRect[i] #regresa el rectángulo que se formó
            #posición arriba a la izquierda
            cv2.rectangle(frame,(int(pos_x),int(pos_y)),(int(pos_x+rectangle_width),int(pos_y+rectangle_width)),color, 2)

            #COORDENADAS
            coordinates = cv2.moments(countours_poly[i])
            if coordinates["m00"]==0:
                oordinates["m00"] = 1
            
            x = int(coordinates["m10"]/coordinates["m00"]) 
            y = int(coordinates["m01"]/coordinates["m00"]) 
            cv2.putText(frame,'{},{}'.format(x,y),(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv2.LINE_AA)

    cv2.imshow("Camera", frame) #Se visualiza la cámara
    cv2.moveWindow("Camera",0,0)
#   cv2.imshow("MascaraBinaria", maskY)
#  cv2.imshow("DibujodeContornos", drawing)





capture.release()
cv2.destroyAllWindows()
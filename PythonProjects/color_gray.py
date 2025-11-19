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

#creando el rectangulo
color = (0, 255,0 ) #BGR 
thickness = -1 #use -1 to fill
size = 50
Pos_x = 200
Pos_y = 200

while True:
    ret, frame = capture.read()

    

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

    x1, y1 = Pos_x-size, Pos_y-size
    x2, y2 = Pos_x-size, Pos_y-size

    

    frame_gray[y1:y2, x1:x2] = frame[y1:y2, x1:x2] #filas y columnas por eso se hace "y" y luego "x"
    # cv2.rectangle(frame_gray, (x1,y1), (x2,y2), color, thickness)


    if cv2.waitKey(1)==ord('q'):
        break


    cv2.imshow("Camera", frame_gray)

capture.release()
cv2.destroyAllWindows()
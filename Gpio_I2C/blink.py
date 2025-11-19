import time
import board
import digitalio

print("hello blinky!")

#Defininimos el PIN y el modo OUTPUT
ledD18 = digitalio.DigitalInOut(board.D18) #Elige el elemento D18
ledD18.direction = digitalio.Direction.OUTPUT #Lo configura como salida

def blinky(led):
    led.value = True #Coloca en alto
    time.sleep(0.5) #Tiempo en segundos
    led.value = False #Coloca en bajo
    time.sleep(0.5) #Tiempo en segundos


while True:
    blinky(ledD18)

  
